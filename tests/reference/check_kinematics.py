"""Step 1 check: C++ ArmKinematics vs mink (the library the python IK uses).

For random joint configurations (plus q = 0 and both joint-limit corners) compares
  * resolved qpos/dof indices and joint limits
  * TCP pose            vs mink.Configuration.get_transform_frame_to_world()
  * TCP body Jacobian   vs mink.Configuration.get_frame_jacobian()  (arm columns)

Run with the python that has openarm_control 0.1.0 + mink (the reference stack):
  <venv>/bin/python check_kinematics.py <ik_kinematics_dump> <scene.xml> [--record <dir>]

--record <dir> also writes <dir>/kinematics.json.gz for tests/check_golden.py.
"""

import gzip
import json
import pathlib
import subprocess
import sys

import mink
import mujoco
import numpy as np

NUM_RANDOM = 1000
POS_TOL_M = 1e-12
ROT_TOL = 1e-12
JAC_TOL = 1e-12
SIDES = ("right", "left")
TCP = {"right": "openarm_right_tcp", "left": "openarm_left_tcp"}


def joint_ids(model, side):
    return [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{side}_joint{i}") for i in range(1, 8)]


def make_samples(model, rng):
    limits = {s: model.jnt_range[joint_ids(model, s)] for s in SIDES}
    samples = [{s: [0.0] * 7 for s in SIDES},
               {s: limits[s][:, 0].tolist() for s in SIDES},
               {s: limits[s][:, 1].tolist() for s in SIDES}]
    for _ in range(NUM_RANDOM):
        samples.append({s: rng.uniform(limits[s][:, 0], limits[s][:, 1]).tolist() for s in SIDES})
    return samples


def reference(model, sample):
    config = mink.Configuration(model)
    q = config.q.copy()
    for side in SIDES:
        q[model.jnt_qposadr[joint_ids(model, side)]] = sample[side]
    config.update(q=q)
    out = {}
    for side in SIDES:
        T = config.get_transform_frame_to_world(TCP[side], "body")
        wxyz_xyz = T.wxyz_xyz
        jac = config.get_frame_jacobian(TCP[side], "body")[:, model.jnt_dofadr[joint_ids(model, side)]]
        out[side] = {"pos": wxyz_xyz[4:], "quat_wxyz": wxyz_xyz[:4], "jac": jac}
    return out


def main() -> int:
    record_dir = None
    if "--record" in sys.argv:
        i = sys.argv.index("--record")
        record_dir = sys.argv[i + 1]
        del sys.argv[i:i + 2]
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    binary, xml = sys.argv[1], sys.argv[2]
    model = mujoco.MjModel.from_xml_path(xml)
    samples = make_samples(model, np.random.default_rng(0))

    stdin = "\n".join(json.dumps(s) for s in samples) + "\n"
    result = subprocess.run([binary, xml], input=stdin, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        print(f"ik_kinematics_dump failed ({result.returncode}):\n{result.stderr}")
        return 1
    lines = result.stdout.splitlines()
    indices, outputs = json.loads(lines[0])["indices"], [json.loads(l) for l in lines[1:]]

    ok = True
    expected_indices = {}
    for side in SIDES:
        ids = joint_ids(model, side)
        expected = {"qpos": model.jnt_qposadr[ids].tolist(), "dof": model.jnt_dofadr[ids].tolist(),
                    "lower": model.jnt_range[ids, 0].tolist(), "upper": model.jnt_range[ids, 1].tolist()}
        expected_indices[side] = expected
        for key, value in expected.items():
            if not np.allclose(indices[side][key], value):
                print(f"FAIL indices[{side}][{key}]: cpp={indices[side][key]} python={value}")
                ok = False
        print(f"{side:5s} indices: qpos={indices[side]['qpos']} dof={indices[side]['dof']} "
              f"tcp_body={indices[side]['tcp_body']}")

    worst = {"pos": 0.0, "rot": 0.0, "jac": 0.0}
    expected = []
    for sample, cpp in zip(samples, outputs, strict=True):
        ref = reference(model, sample)
        expected.append({side: {"pos": ref[side]["pos"].tolist(), "quat_wxyz": ref[side]["quat_wxyz"].tolist(),
                                "jac": ref[side]["jac"].ravel().tolist()} for side in SIDES})
        for side in SIDES:
            worst["pos"] = max(worst["pos"], np.max(np.abs(np.array(cpp[side]["pos"]) - ref[side]["pos"])))
            dot = abs(float(np.dot(cpp[side]["quat_wxyz"], ref[side]["quat_wxyz"])))
            worst["rot"] = max(worst["rot"], 1.0 - min(dot, 1.0))
            jac = np.array(cpp[side]["jac"]).reshape(6, 7)
            worst["jac"] = max(worst["jac"], np.max(np.abs(jac - ref[side]["jac"])))
    passed = worst["pos"] <= POS_TOL_M and worst["rot"] <= ROT_TOL and worst["jac"] <= JAC_TOL
    ok &= passed
    if record_dir is not None:
        path = pathlib.Path(record_dir) / "kinematics.json.gz"
        golden = {"part": "kinematics", "reference": "mink 1.1 (python)",
                  "tol": {"pos": POS_TOL_M, "rot": ROT_TOL, "jac": JAC_TOL},
                  "indices": expected_indices, "records": samples, "expected": expected}
        with gzip.open(path, "wt") as f:
            json.dump(golden, f, separators=(",", ":"))
        print(f"recorded {len(samples)} configs → {path}")
    print(f"{'PASS' if passed else 'FAIL'}: {len(samples)} configs x 2 arms | "
          f"max |dp|={worst['pos']:.1e} m, max (1-|q·q'|)={worst['rot']:.1e}, max |dJ|={worst['jac']:.1e}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
