"""Parity test: C++ quest_replay vs the python receiver logic it was ported from.

Generates a synthetic Quest packet stream (500 Hz ticks re-reading ~80 Hz packets,
grip engage/release, an INVALID window, a STALE window), runs it through
  * the real python QuestPoseProcessor + OneEuroPoseSmoother from
    vendor/dora-openarm-vr, driven by a copy of quest_receiver._run()'s per-tick logic
  * the C++ quest_replay binary
and compares every tick.

Usage: python parity_check.py <path/to/quest_replay>
"""

import json
import pathlib
import subprocess
import sys
import tempfile
import types

import numpy as np
from scipy.spatial.transform import Rotation

HERE = pathlib.Path(__file__).resolve().parent
VENDOR_SRC = HERE.parent.parent / "vendor" / "dora-openarm-vr" / "src"

TICK_S = 0.002          # dora/timer/millis/2
PACKET_PERIOD_S = 0.0125  # ~80 Hz headset send rate
DURATION_S = 8.0
POS_TOL_M = 1e-5        # python works in float32
ROT_TOL_RAD = 1e-4


def import_python_reference():
    # quest_receiver.py imports dora/rclpy/std_srvs at module level; stub them so
    # the pure-python pieces can be imported without a ROS 2 / dora runtime.
    for name in ("dora", "rclpy", "std_srvs", "std_srvs.srv"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["std_srvs.srv"].SetBool = object
    sys.path.insert(0, str(VENDOR_SRC))
    from dora_openarm_vr import quest_receiver
    from dora_openarm_vr.smoothing import OneEuroPoseSmoother

    quest_receiver._save_reference = lambda _ref: None  # don't touch ~/.cache
    return quest_receiver, OneEuroPoseSmoother


def unity_pose(pos, rot: Rotation) -> dict:
    q = rot.as_quat()  # x, y, z, w
    return {"x": pos[0], "y": pos[1], "z": pos[2], "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]}


def generate_stream(with_reference: bool, seed: int) -> list[dict]:
    rng = np.random.default_rng(seed)
    # Real captured reference (note qw < 0: exercises quaternion sign handling).
    reference = {"x": -0.0383, "y": 1.5121, "z": -0.2395,
                 "qx": 0.0242, "qy": -0.1368, "qz": -0.0182, "qw": -0.9901}
    records, packet, next_packet_t = [], None, 0.0
    for k in range(int(DURATION_S / TICK_S)):
        t = k * TICK_S
        if t >= next_packet_t:
            next_packet_t += PACKET_PERIOD_S
            w = 2.0 * np.pi * t
            right_p = [0.25 + 0.15 * np.sin(0.5 * w), 1.20 + 0.10 * np.sin(0.3 * w), -0.45 + 0.1 * np.cos(0.4 * w)]
            left_p = [-0.25 + 0.1 * np.cos(0.6 * w), 1.15 + 0.05 * np.sin(0.7 * w), -0.40]
            right_p = np.array(right_p) + rng.normal(0.0, 0.002, 3)
            left_p = np.array(left_p) + rng.normal(0.0, 0.002, 3)
            right_r = Rotation.from_euler("xyz", [0.4 * np.sin(0.3 * w), 0.6 * np.sin(0.2 * w), 0.3 * t])
            left_r = Rotation.from_euler("xyz", [-0.2, 0.5 * np.cos(0.25 * w), -0.2 * t])
            if rng.random() < 0.5:  # random quaternion sign flips, same rotation
                right_r = Rotation.from_quat(-right_r.as_quat())
            packet = {
                "t": t,
                "rc": unity_pose(right_p, right_r),
                "lc": unity_pose(left_p, left_r),
                "rt": float(rng.random()), "lt": float(rng.random()),
                "rg": 1.0 if (0.5 <= t < 3.0 or 4.0 <= t < 7.5) else 0.0,
                "lg": 1.0 if 1.0 <= t < 6.0 else 0.0,
                "lsx": 0.1, "lsy": -0.2, "rsx": 0.3, "rsy": 0.4,
                "a": False, "b": True, "x": 0, "y": 1,
                "v": 1 if 5.0 <= t < 5.2 else 0,
                "vr": 2 if 2.0 <= t < 2.3 else 0,
                "vl": 0,
            }
            if with_reference:
                packet["rf"] = reference
        records.append({"t": t, "msg": packet})
    return records


def run_python(records, quest_receiver, OneEuroPoseSmoother):
    """Copy of the per-tick logic of quest_receiver._run(), minus dora/ROS I/O."""
    processor = quest_receiver.QuestPoseProcessor()
    arms = {}
    for side in ("right", "left"):
        arms[side] = {
            "smoother": OneEuroPoseSmoother(min_cutoff=2.0, beta=0.04, d_cutoff=1.5),
            "prev_v": 0, "prev_engaged": False, "hold_start": None,
            "fired": False, "last_sent": None,
        }
    outputs = []
    for rec in records:
        now, msg = rec["t"], rec["msg"]
        raw_right, raw_left = processor.process(msg)
        frame = {}
        for side, raw, gkey, vkey in (("right", raw_right, "rg", "vr"), ("left", raw_left, "lg", "vl")):
            a = arms[side]
            engaged = float(msg.get(gkey, 0.0)) > 0.5
            rising = engaged and not a["prev_engaged"]
            if rising:
                a["smoother"].reset()
                a["hold_start"], a["fired"] = now, False
            elif not a["fired"] and a["hold_start"] is not None and now - a["hold_start"] >= 5.0:
                a["fired"] = True
            v = int(msg.get(vkey, 0))
            if v == 2:
                if a["prev_v"] != 2:
                    a["smoother"].reset()
                pose = None
            else:
                pose = a["smoother"].smooth(now, raw)
            a["prev_v"], a["prev_engaged"] = v, engaged
            if engaged and pose is not None:
                a["last_sent"] = pose
            frame[side] = {
                "pose": None if a["last_sent"] is None else [float(x) for x in a["last_sent"]],
                "engaged": engaged,
                "timer_running": a["hold_start"] is not None and not a["fired"],
            }
        outputs.append(frame)
    return outputs


def run_cpp(records, binary):
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
        for rec in records:
            f.write(json.dumps(rec) + "\n")
        path = f.name
    result = subprocess.run([binary, path], capture_output=True, text=True, check=False)
    pathlib.Path(path).unlink(missing_ok=True)
    if result.returncode != 0:
        sys.exit(f"quest_replay failed ({result.returncode}):\n{result.stderr}")
    return [json.loads(line) for line in result.stdout.splitlines()]


def compare(name, py_out, cpp_out) -> bool:
    if len(py_out) != len(cpp_out):
        print(f"[{name}] FAIL: tick count python={len(py_out)} cpp={len(cpp_out)}")
        return False
    worst_pos, worst_rot, mismatches, posed = 0.0, 0.0, [], 0
    for k, (py, cc) in enumerate(zip(py_out, cpp_out)):
        for side in ("right", "left"):
            p, c = py[side], cc[side]
            for flag in ("engaged", "timer_running"):
                if p[flag] != c[flag]:
                    mismatches.append(f"tick {k} {side}.{flag}: py={p[flag]} cpp={c[flag]}")
            if (p["pose"] is None) != (c["pose"] is None):
                mismatches.append(f"tick {k} {side}.pose presence: py={p['pose']} cpp={c['pose']}")
                continue
            if p["pose"] is None:
                continue
            posed += 1
            pp, cp = np.array(p["pose"]), np.array(c["pose"])
            worst_pos = max(worst_pos, float(np.max(np.abs(pp[:3] - cp[:3]))))
            qp, qc = pp[3:] / np.linalg.norm(pp[3:]), cp[3:] / np.linalg.norm(cp[3:])
            angle = 2.0 * np.arccos(np.clip(abs(float(qp @ qc)), 0.0, 1.0))  # sign-agnostic
            worst_rot = max(worst_rot, angle)
    ok = not mismatches and worst_pos <= POS_TOL_M and worst_rot <= ROT_TOL_RAD
    print(f"[{name}] {'PASS' if ok else 'FAIL'}: {len(py_out)} ticks, {posed} posed arm-ticks, "
          f"max |dp|={worst_pos:.2e} m, max dθ={worst_rot:.2e} rad, flag mismatches={len(mismatches)}")
    for line in mismatches[:10]:
        print("   ", line)
    return ok


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    binary = sys.argv[1]
    quest_receiver, smoother_cls = import_python_reference()
    ok = True
    for name, with_ref, seed in (("with-reference", True, 1), ("no-reference", False, 2)):
        records = generate_stream(with_ref, seed)
        ok &= compare(name, run_python(records, quest_receiver, smoother_cls), run_cpp(records, binary))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
