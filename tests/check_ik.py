"""IK steps 2-4: C++ (ik_check) vs the python stack the robot ran.

  se3     step 2: log / rminus / inverse-multiply / jlog vs mink's native lie ops
  boxqp   step 3: box-constrained QP vs DAQP (the backend openarm_control uses)
  replay  step 4: full IkSolver vs openarm_control 0.1.0 _IKSolver, tick by tick

Run with the python that has openarm_control 0.1.0 + mink 1.1 + daqp:
  /home/tan/work/ws/dora-openarm-ros2/.venv/bin/python check_ik.py <part> <ik_check> <scene.xml>
"""

import contextlib
import io
import json
import subprocess
import sys

import mujoco
import numpy as np

NUM_JOINTS = 7
SIDES = ("right", "left")
RNG = np.random.default_rng(0)


def run_cpp(binary, args, records):
    stdin = "".join(json.dumps(r) + "\n" for r in records)
    result = subprocess.run([binary, *args], input=stdin, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        sys.exit(f"ik_check {' '.join(args)} failed ({result.returncode}):\n{result.stderr}")
    return [json.loads(line) for line in result.stdout.splitlines()]


def report(name, worst, tol):
    ok = worst <= tol
    print(f"{'PASS' if ok else 'FAIL'}: {name}: max error {worst:.2e} (tol {tol:.0e})")
    return ok


# ── step 2 ────────────────────────────────────────────────────────────────────
def random_quat(angle=None):
    axis = RNG.normal(size=3)
    axis /= np.linalg.norm(axis)
    if angle is None:
        angle = RNG.uniform(-np.pi, np.pi)
    q = np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis])
    return q if RNG.random() < 0.5 else -q  # both signs of the same rotation


def random_pose(angle=None):
    return np.concatenate([random_quat(angle), RNG.uniform(-1, 1, 3)]).tolist()


def check_se3(binary):
    from mink.lie import _lie_ops_c as native

    special = [0.0, 1e-12, 1e-9, 1e-6, 1e-3, np.pi - 1e-3, np.pi - 1e-7, np.pi]
    records = [{"a": random_pose(a), "b": random_pose()} for a in special]
    records += [{"a": random_pose(), "b": random_pose(a)} for a in special]
    records += [{"a": random_pose(), "b": random_pose()} for _ in range(2000)]
    outputs = run_cpp(binary, ["se3"], records)

    worst = {"log": 0.0, "rminus": 0.0, "inv_mul": 0.0, "jlog": 0.0}
    for rec, out in zip(records, outputs, strict=True):
        a, b = np.array(rec["a"]), np.array(rec["b"])
        worst["log"] = max(worst["log"], np.abs(np.array(out["log_a"]) - native.se3_log(a)).max())
        worst["rminus"] = max(worst["rminus"], np.abs(np.array(out["rminus"]) - native.se3_rminus(a, b)).max())
        ref = native.se3_inverse_multiply(a, b)
        got = np.array(out["inv_mul"])
        sign = 1.0 if np.dot(got[:4], ref[:4]) >= 0 else -1.0  # q ≡ −q
        got[:4] *= sign
        worst["inv_mul"] = max(worst["inv_mul"], np.abs(got - ref).max())
        jlog = np.array(out["jlog_a"]).reshape(6, 6)
        worst["jlog"] = max(worst["jlog"], np.abs(jlog - native.se3_jlog(a)).max())
    # Same formulas in the same order as mink's C code → agreement to ~1e-14,
    # including the θ → 0 Taylor branches and θ = π.
    tol = 1e-12
    print(f"{len(records)} pose pairs (incl. θ ∈ {{0, 1e-12 … π}})")
    return all([report(f"se3 {k}", v, tol) for k, v in worst.items()])


# ── step 3 ────────────────────────────────────────────────────────────────────
def random_boxqp():
    n = NUM_JOINTS
    a = RNG.normal(size=(6, n)) * RNG.uniform(0.1, 1.0)
    h = a.T @ a + RNG.uniform(1e-3, 0.3) * np.eye(n)  # like JᵀJ + damping
    c = RNG.normal(size=n) * RNG.uniform(0.01, 1.0)
    lo = -RNG.uniform(0.0, 0.5, n)
    hi = RNG.uniform(0.0, 0.5, n)
    kind = RNG.integers(0, 4)
    if kind == 1:  # joint already beyond its lower limit → bound forces motion
        i = RNG.integers(n)
        lo[i] = RNG.uniform(0.01, 0.1)
        hi[i] = lo[i] + RNG.uniform(0.0, 0.5)
    elif kind == 2:  # unlimited joints
        mask = RNG.random(n) < 0.4
        lo[mask], hi[mask] = -np.inf, np.inf
    elif kind == 3:  # joint exactly at a limit
        i = RNG.integers(n)
        lo[i] = 0.0
    return h, c, lo, hi


def check_boxqp(binary):
    import qpsolvers

    problems = [random_boxqp() for _ in range(3000)]
    records = [{"H": h.ravel().tolist(), "c": c.tolist(),
                "lo": [x if np.isfinite(x) else (-1e300 if x < 0 else 1e300) for x in lo],
                "hi": [x if np.isfinite(x) else (-1e300 if x < 0 else 1e300) for x in hi]}
               for h, c, lo, hi in problems]
    outputs = run_cpp(binary, ["boxqp"], records)

    worst, active_total, failures = 0.0, 0, 0
    for (h, c, lo, hi), out in zip(problems, outputs, strict=True):
        if out["status"] != "optimal":
            failures += 1
            continue
        # Same formulation as mink: G Δq ≤ h with G = [I; -I] on the limited DOFs.
        lim = np.isfinite(lo)
        g = np.vstack([np.eye(NUM_JOINTS)[lim], -np.eye(NUM_JOINTS)[lim]])
        hv = np.hstack([hi[lim], -lo[lim]])
        ref = qpsolvers.solve_qp(h, c, g if lim.any() else None, hv if lim.any() else None, solver="daqp")
        x = np.array(out["x"])
        worst = max(worst, np.abs(x - ref).max())
        active_total += int(np.sum(np.isclose(x, lo) | np.isclose(x, hi)))
    print(f"{len(problems)} QPs, {active_total} active bounds in total, {failures} not optimal")
    return report("box QP vs DAQP", worst, 1e-9) and failures == 0


# ── step 4 ────────────────────────────────────────────────────────────────────
def make_targets(model):
    """Smooth reachable joint-space motion → FK targets, plus nasty segments."""
    data = mujoco.MjData(model)
    num_ticks = 600
    ids = {s: [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{s}_joint{i}")
               for i in range(1, NUM_JOINTS + 1)] for s in SIDES}
    tcp = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{s}_tcp") for s in SIDES}
    rng = model.jnt_range
    mid = {s: rng[ids[s]].mean(axis=1) for s in SIDES}
    amp = {s: 0.35 * (rng[ids[s], 1] - rng[ids[s], 0]) / 2 for s in SIDES}
    freq = {s: RNG.uniform(0.5, 2.0, NUM_JOINTS) for s in SIDES}
    phase = {s: RNG.uniform(0, 2 * np.pi, NUM_JOINTS) for s in SIDES}

    records = []
    for t in range(num_ticks):
        for s in SIDES:
            q = mid[s] + amp[s] * np.sin(2 * np.pi * freq[s] * t / num_ticks + phase[s])
            data.qpos[model.jnt_qposadr[ids[s]]] = q
        mujoco.mj_kinematics(model, data)
        rec = {}
        for s in SIDES:
            pos = data.xpos[tcp[s]].copy()
            quat = data.xquat[tcp[s]].copy()
            if s == "right" and 200 <= t < 260:  # unreachable: 0.6 m further forward
                pos[0] += 0.6
            if s == "left" and 300 <= t < 330:  # 150° orientation jump
                rot = np.empty(4)
                mujoco.mju_mulQuat(rot, quat, np.array([np.cos(1.31), np.sin(1.31), 0.0, 0.0]))
                quat = rot
            rec[s] = np.concatenate([pos, quat]).tolist()  # [px,py,pz,qw,qx,qy,qz]
        if 400 <= t < 420:  # left controller silent → solver must wait
            del rec["left"]
        if t % 7 == 3:  # right arrives one tick later than left now and then
            rec.pop("right", None)
        records.append(rec)
    return records


def check_replay(binary, xml):
    from openarm_control.config import ArmSetup
    from openarm_control.kinematics import IKParams, Kinematics

    # The dataflow yaml's ik args (port_to_C++ IkParams defaults).
    params = IKParams(position_cost=1.0, orientation_cost=1.0, lm_damping=0.01, damping=0.1,
                      posture_cost=0.01, max_iters=10, dt=0.1)
    with contextlib.redirect_stdout(io.StringIO()):  # _IKSolver prints debug sets
        setup = ArmSetup.from_args(xml=xml, mode="bimanual",
                                   frame_right="openarm_right_tcp", frame_type_right="body",
                                   frame_left="openarm_left_tcp", frame_type_left="body",
                                   keyframe="home")
        kin = Kinematics(setup, params)
    model = setup.model
    qadr = {s: model.jnt_qposadr[[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                                     f"openarm_{s}_joint{i}")
                                   for i in range(1, NUM_JOINTS + 1)]] for s in SIDES}

    records = make_targets(model)
    outputs = run_cpp(binary, ["replay", xml], records)

    worst, status_mismatch, solved = 0.0, 0, 0
    worst_tick = -1
    for t, (rec, out) in enumerate(zip(records, outputs, strict=True)):
        for s in SIDES:
            if s in rec:
                kin.set_target(s, np.array(rec[s]))
        if kin.ready():
            with contextlib.redirect_stdout(io.StringIO()):
                res = kin.solve()
            py_status = "solved" if res is not None else "QP failed"
        else:
            py_status = "not ready"
        solved += py_status == "solved"
        status_mismatch += py_status != out["status"]
        q_py = kin._ik._config.data.qpos
        for s in SIDES:
            err = np.abs(np.array(out[s]) - q_py[qadr[s]]).max()
            if err > worst:
                worst, worst_tick = err, t
    print(f"{len(records)} ticks, {solved} solves, {status_mismatch} status mismatches, "
          f"worst joint error at tick {worst_tick}")
    return report("IkSolver vs openarm_control 0.1.0 (rad)", worst, 1e-8) and status_mismatch == 0


def main() -> int:
    if len(sys.argv) != 4 or sys.argv[1] not in ("se3", "boxqp", "replay"):
        print(__doc__)
        return 2
    part, binary, xml = sys.argv[1:]
    ok = {"se3": lambda: check_se3(binary),
          "boxqp": lambda: check_boxqp(binary),
          "replay": lambda: check_replay(binary, xml)}[part]()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
