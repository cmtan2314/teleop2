"""Replays a golden file through a C++ check binary and compares the outputs.

The golden files in tests/data/ hold the inputs and the outputs of the python
code this port replaces (dora-openarm-vr receiver, openarm_control 0.1.0 + mink
1.1 + daqp IK, dora_ros2_bridge), recorded once with tests/reference/*.py
--record. This script needs only the python standard library, so the tests run
without any of that installed.

  check_golden.py <golden.json.gz> <binary> [binary args...]

  kinematics.json.gz   ik_kinematics_dump <scene.xml>
  se3.json.gz          ik_check se3
  boxqp.json.gz        ik_check boxqp <daqp|activeset>
  replay.json.gz       ik_check replay <scene.xml> <daqp|activeset> <single|threads>
  quest.json.gz        quest_replay
  bridge.json.gz       bridge_replay --grasp-presets <grasp_presets.yaml>
"""

import gzip
import json
import math
import os
import subprocess
import sys
import tempfile

NUM_JOINTS = 7
ARM_TWIST_DIM = 6
QUAT_DIM = 4
POSITION_DIM = 3
SIDES = ("right", "left")


def max_abs_diff(a, b):
    if len(a) != len(b):
        sys.exit(f"length mismatch: {len(a)} vs {len(b)}")
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)


def report(name, worst, tol):
    ok = worst <= tol
    print(f"{'PASS' if ok else 'FAIL'}: {name}: max error {worst:.2e} (tol {tol:.0e})")
    return ok


def run_binary(command, records):
    stdin = "".join(json.dumps(r) + "\n" for r in records)
    result = subprocess.run(command, input=stdin, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        sys.exit(f"{' '.join(command)} failed ({result.returncode}):\n{result.stderr}")
    return [json.loads(line) for line in result.stdout.splitlines()]


def check_se3(golden, command):
    outputs = run_binary(command, golden["records"])
    worst = {"log_a": 0.0, "rminus": 0.0, "inv_mul": 0.0, "jlog_a": 0.0}
    for out, ref in zip(outputs, golden["expected"], strict=True):
        for key in ("log_a", "rminus", "jlog_a"):
            worst[key] = max(worst[key], max_abs_diff(out[key], ref[key]))
        got = list(out["inv_mul"])
        if sum(g * r for g, r in zip(got[:QUAT_DIM], ref["inv_mul"][:QUAT_DIM])) < 0.0:
            got[:QUAT_DIM] = [-g for g in got[:QUAT_DIM]]  # q ≡ −q
        worst["inv_mul"] = max(worst["inv_mul"], max_abs_diff(got, ref["inv_mul"]))
    print(f"{len(outputs)} pose pairs")
    return all([report(f"se3 {k}", v, golden["tol"]) for k, v in worst.items()])


def check_boxqp(golden, command):
    outputs = run_binary(command, golden["records"])
    worst, failures = 0.0, 0
    for out, ref in zip(outputs, golden["expected"], strict=True):
        if out["status"] != "optimal":
            failures += 1
            continue
        worst = max(worst, max_abs_diff(out["x"], ref["x"]))
    print(f"{len(outputs)} QPs, {failures} not optimal")
    return report("box QP vs DAQP", worst, golden["tol"]) and failures == 0


def check_replay(golden, command):
    outputs = run_binary(command, golden["records"])
    per_joint = {s: [0.0] * NUM_JOINTS for s in SIDES}
    status_mismatch, solved = 0, 0
    for out, ref in zip(outputs, golden["expected"], strict=True):
        solved += ref["status"] == "solved"
        status_mismatch += out["status"] != ref["status"]
        for s in SIDES:
            if len(out[s]) != NUM_JOINTS:
                sys.exit(f"replay output for {s} has {len(out[s])} joints, expected {NUM_JOINTS}")
            per_joint[s] = [max(w, abs(c - p)) for w, c, p in zip(per_joint[s], out[s], ref[s])]
    print(f"{len(outputs)} ticks, {solved} solves, {status_mismatch} status mismatches")
    print("max |q_cpp - q_py| per joint (rad):")
    print("        " + "".join(f"{'j' + str(i + 1):>10}" for i in range(NUM_JOINTS)))
    for s in SIDES:
        print(f"  {s:<6}" + "".join(f"{e:10.1e}" for e in per_joint[s]))
    worst = max(max(v) for v in per_joint.values())
    return report("IkSolver vs openarm_control 0.1.0 (rad)", worst, golden["tol"]) and status_mismatch == 0


def check_kinematics(golden, command):
    outputs = run_binary(command, golden["records"])
    indices, outputs = outputs[0]["indices"], outputs[1:]
    tol = golden["tol"]
    ok = True
    for s in SIDES:
        ref = golden["indices"][s]
        for key in ("qpos", "dof"):
            if indices[s][key] != ref[key]:
                print(f"FAIL indices[{s}][{key}]: cpp={indices[s][key]} python={ref[key]}")
                ok = False
        for key in ("lower", "upper"):
            if max_abs_diff(indices[s][key], ref[key]) > tol["pos"]:
                print(f"FAIL indices[{s}][{key}]: cpp={indices[s][key]} python={ref[key]}")
                ok = False
    worst = {"pos": 0.0, "rot": 0.0, "jac": 0.0}
    for out, ref in zip(outputs, golden["expected"], strict=True):
        for s in SIDES:
            worst["pos"] = max(worst["pos"], max_abs_diff(out[s]["pos"], ref[s]["pos"]))
            dot = abs(sum(a * b for a, b in zip(out[s]["quat_wxyz"], ref[s]["quat_wxyz"])))
            worst["rot"] = max(worst["rot"], 1.0 - min(dot, 1.0))
            if len(out[s]["jac"]) != ARM_TWIST_DIM * NUM_JOINTS:
                sys.exit(f"jacobian for {s} has {len(out[s]['jac'])} entries")
            worst["jac"] = max(worst["jac"], max_abs_diff(out[s]["jac"], ref[s]["jac"]))
    print(f"{len(outputs)} configs x 2 arms")
    names = {"pos": "TCP position (m)", "rot": "TCP orientation (1-|q·q'|)", "jac": "body Jacobian"}
    return all([report(names[k], v, tol[k]) for k, v in worst.items()]) and ok


def check_quest(golden, command):
    """quest_replay reads a JSONL file (path as last argument), one line per tick."""
    tol = golden["tol"]
    ok = True
    for scenario in golden["scenarios"]:
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
            for rec in scenario["records"]:
                f.write(json.dumps(rec) + "\n")
            path = f.name
        try:
            outputs = run_binary([*command, path], [])
        finally:
            os.unlink(path)
        expected = scenario["expected"]
        if len(outputs) != len(expected):
            print(f"[{scenario['name']}] FAIL: tick count python={len(expected)} cpp={len(outputs)}")
            ok = False
            continue
        worst_pos, worst_rot, mismatches, posed = 0.0, 0.0, [], 0
        for k, (ref, out) in enumerate(zip(expected, outputs)):
            for side in SIDES:
                p, c = ref[side], out[side]
                for flag in ("engaged", "timer_running"):
                    if p[flag] != c[flag]:
                        mismatches.append(f"tick {k} {side}.{flag}: py={p[flag]} cpp={c[flag]}")
                if (p["pose"] is None) != (c["pose"] is None):
                    mismatches.append(f"tick {k} {side}.pose presence differs")
                    continue
                if p["pose"] is None:
                    continue
                posed += 1
                worst_pos = max(worst_pos, max_abs_diff(p["pose"][:POSITION_DIM], c["pose"][:POSITION_DIM]))
                qp, qc = p["pose"][POSITION_DIM:], c["pose"][POSITION_DIM:]
                dot = abs(sum(a * b for a, b in zip(qp, qc))) / (math.hypot(*qp) * math.hypot(*qc))
                worst_rot = max(worst_rot, 2.0 * math.acos(min(dot, 1.0)))  # sign-agnostic angle
        passed = not mismatches and worst_pos <= tol["pos"] and worst_rot <= tol["rot"]
        print(f"[{scenario['name']}] {'PASS' if passed else 'FAIL'}: {len(outputs)} ticks, "
              f"{posed} posed arm-ticks, max |dp|={worst_pos:.2e} m, max dθ={worst_rot:.2e} rad, "
              f"flag mismatches={len(mismatches)}")
        for line in mismatches[:10]:
            print("   ", line)
        ok &= passed
    return ok


def check_bridge(golden, command):
    """Every published message (topic, joint names, values) must match, in order."""
    ok = True
    for scenario in golden["scenarios"]:
        outputs = run_binary([*command, *scenario["args"]], golden["records"])
        expected = scenario["expected"]
        name = scenario["name"]
        if len(outputs) != len(expected):
            print(f"FAIL {name}: python published {len(expected)} messages, C++ {len(outputs)}")
            ok = False
            continue
        worst, by_topic, passed = 0.0, {}, True
        for i, ((topic, names, data), out) in enumerate(zip(expected, outputs)):
            if out["topic"] != topic or out["names"] != names or len(out["data"]) != len(data):
                print(f"FAIL {name}: message {i}: python {topic} {names} vs C++ {out['topic']} {out['names']}")
                passed = False
                break
            worst = max(worst, max_abs_diff(out["data"], data))
            by_topic[topic] = by_topic.get(topic, 0) + 1
        passed = passed and worst <= golden["tol"]
        print(f"{'PASS' if passed else 'FAIL'} {name}: {len(expected)} messages, max value diff {worst:.1e}")
        for topic, count in sorted(by_topic.items()):
            print(f"      {count:6d}  {topic}")
        ok &= passed
    return ok


CHECKS = {"se3": check_se3, "boxqp": check_boxqp, "replay": check_replay,
          "kinematics": check_kinematics, "quest": check_quest, "bridge": check_bridge}


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    with gzip.open(sys.argv[1], "rt") as f:
        golden = json.load(f)
    part = golden.get("part")
    if part not in CHECKS:
        sys.exit(f"unknown golden part '{part}' in {sys.argv[1]}")
    print(f"golden {part}: reference = {golden['reference']}")
    return 0 if CHECKS[part](golden, sys.argv[2:]) else 1


if __name__ == "__main__":
    sys.exit(main())
