"""teleop_bridge (C++) vs src/dora_ros2_bridge/main.py (python dora-to-ros2).

Runs the real python `_run()` with dora / rclpy / ROS message packages
replaced by stubs: the stub dora.Node yields a scripted event stream, the stub
publishers record every message. The same stream goes through the C++
bridge_replay harness, and every published message (topic, joint names,
values) must match. Low-pass filters are off in both (not ported).

Run with the dora-openarm-ros2 venv python (numpy, pyarrow, yaml):
  <repo>/.venv/bin/python check_bridge.py <bridge_replay> <dora-openarm-ros2 repo> [--record <dir>]

--record <dir> also writes <dir>/bridge.json.gz for tests/check_golden.py.
"""

import argparse
import contextlib
import gzip
import io
import json
import pathlib
import subprocess
import sys
import threading
import types

import numpy as np
import pyarrow as pa

REPO = None     # dora-openarm-ros2 checkout, set from argv
PRESETS = None  # its config/grasp_presets.yaml
RNG = np.random.default_rng(0)
SIDES = ("left", "right")


# ── stubs ─────────────────────────────────────────────────────────────────────
class _Msg:
    """Auto-vivifying message: msg.header.stamp = … just works."""

    def __init__(self, *args, **kwargs):
        self.__dict__.update(kwargs)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = _Msg()
        setattr(self, name, value)
        return value


class _Recorder:
    def __init__(self, topic, sink):
        self.topic, self.sink = topic, sink

    def publish(self, msg):
        self.sink.append((self.topic, msg))


class _StubNode:
    published = []
    subs, services, timers = {}, {}, []

    def __init__(self, name):
        pass

    def create_publisher(self, _type, topic, _qos):
        return _Recorder(topic, _StubNode.published)

    def create_subscription(self, _type, topic, cb, _qos):
        _StubNode.subs[topic] = cb

    def create_service(self, _type, name, cb):
        _StubNode.services[name] = cb

    def create_timer(self, _period, cb):
        _StubNode.timers.append(cb)


def install_stubs(events):
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    msg_classes = {n: type(n, (_Msg,), {}) for n in (
        "Time", "Duration", "JointTrajectoryControllerState", "PoseStamped", "Joy",
        "JointState", "JointTrajectory", "JointTrajectoryPoint", "QoSProfile")}
    set_bool = type("SetBool", (), {"Request": _Msg, "Response": _Msg})

    class DoraNode:
        def __iter__(self):
            for ev in events:
                name, value = ev["ev"], ev.get("v")
                if name == "cmd_timer":
                    for cb in _StubNode.timers:
                        cb()
                elif name.startswith("feedback_"):
                    side = name.removeprefix("feedback_")
                    msg = _Msg()
                    msg.feedback.positions = list(value)
                    cb = _StubNode.subs.get(f"/{side}_joint_trajectory_controller/controller_state")
                    if cb is not None:  # --mode sim has no feedback subscription
                        cb(msg)
                elif name.startswith("grip_"):
                    kind, side = name.removeprefix("grip_").split("_")  # engaged|timer, side
                    srv = _StubNode.services.get(f"/dora_bridge/set_grip_{kind}_{side}")
                    if srv is not None:  # --mode sim has no grip services
                        srv(_Msg(data=value), _Msg())
                else:
                    if name.startswith("button_"):
                        arr = pa.array([bool(value)], type=pa.bool_())
                    elif isinstance(value, list):
                        arr = pa.array(value, type=pa.float32())
                    else:
                        arr = pa.array([value], type=pa.float32())
                    yield {"type": "INPUT", "id": name, "value": arr}

    module("dora", Node=DoraNode)
    module("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None)
    module("rclpy.node", Node=_StubNode)
    module("rclpy.qos", QoSProfile=msg_classes["QoSProfile"],
           HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
           ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2))
    module("builtin_interfaces")
    module("builtin_interfaces.msg", Time=msg_classes["Time"], Duration=msg_classes["Duration"])
    module("control_msgs")
    module("control_msgs.msg",
           JointTrajectoryControllerState=msg_classes["JointTrajectoryControllerState"])
    module("geometry_msgs")
    module("geometry_msgs.msg", PoseStamped=msg_classes["PoseStamped"])
    module("sensor_msgs")
    module("sensor_msgs.msg", Joy=msg_classes["Joy"], JointState=msg_classes["JointState"])
    module("std_srvs")
    module("std_srvs.srv", SetBool=set_bool)
    module("trajectory_msgs")
    module("trajectory_msgs.msg", JointTrajectory=msg_classes["JointTrajectory"],
           JointTrajectoryPoint=msg_classes["JointTrajectoryPoint"])


def python_run(events, mode, cmd_rate_hz):
    _StubNode.published, _StubNode.subs, _StubNode.services, _StubNode.timers = [], {}, {}, []
    install_stubs(events)
    sys.path.insert(0, str(REPO / "src"))
    sys.modules.pop("dora_ros2_bridge.main", None)
    import dora_ros2_bridge.main as bridge  # noqa: PLC0415

    # The spin thread is not needed: subscriptions/services/timers are driven
    # synchronously by the stub dora.Node.
    bridge.threading = types.SimpleNamespace(
        Thread=lambda *a, **k: types.SimpleNamespace(start=lambda: None), Lock=threading.Lock)
    args = argparse.Namespace(mode=mode, grasp_presets=str(PRESETS), lpf_arm_hz=0.0,
                              lpf_hand_hz=0.0, cmd_rate_hz=cmd_rate_hz, cmd_lpf_hz=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        bridge._run(args)
    out = []
    for topic, msg in _StubNode.published:
        if topic == "/vr_buttons":
            out.append((topic, [], [float(b) for b in msg.buttons]))
        elif topic == "/joint_command":
            out.append((topic, list(msg.name), list(msg.position)))
        else:
            out.append((topic, list(msg.joint_names), list(msg.points[0].positions)))
    return out


def cpp_run(binary, events, mode, cmd_rate_hz):
    stdin = "".join(json.dumps(e) + "\n" for e in events)
    result = subprocess.run([binary, "--mode", mode, "--grasp-presets", str(PRESETS),
                             "--cmd-rate-hz", str(cmd_rate_hz)],
                            input=stdin, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        sys.exit(f"bridge_replay failed ({result.returncode}):\n{result.stderr}")
    return [(r["topic"], r["names"], r["data"]) for r in map(json.loads, result.stdout.splitlines())]


# ── scenario ──────────────────────────────────────────────────────────────────
def f32(x):
    """Round like the float32 dora arrays do, so both sides see identical inputs."""
    return [float(v) for v in np.asarray(x, dtype=np.float32)]


def make_events(num_ticks=4000):
    events = []
    target = {s: RNG.uniform(-0.5, 0.5, 7) for s in SIDES}
    physical = {s: np.zeros(7) for s in SIDES}
    engaged = {s: False for s in SIDES}
    timer_left = {s: 0 for s in SIDES}
    buttons = {b: False for b in "abxy"}
    for t in range(num_ticks):
        for s in SIDES:
            # Grip presses/releases; the 5 s window ≈ 150 ticks here.
            if RNG.random() < 0.01:
                engaged[s] = not engaged[s]
                events.append({"ev": f"grip_engaged_{s}", "v": engaged[s]})
                if engaged[s]:
                    timer_left[s] = 150
                    events.append({"ev": f"grip_timer_{s}", "v": True})
            if timer_left[s] > 0:
                timer_left[s] -= 1
                if timer_left[s] == 0:
                    events.append({"ev": f"grip_timer_{s}", "v": False})
            # Robot follows loosely; sometimes jumps (resync branch).
            if t > 50 and RNG.random() < 0.7:
                physical[s] += 0.3 * (target[s] - physical[s]) + RNG.normal(0, 0.003, 7)
                if RNG.random() < 0.01:
                    physical[s] += RNG.normal(0, 0.2, 7)
                events.append({"ev": f"feedback_{s}", "v": f32(physical[s])})
            target[s] += RNG.normal(0, 0.01, 7)
            if RNG.random() < 0.005:
                target[s] += RNG.normal(0, 0.3, 7)  # big target jump
        events.append({"ev": "right_position", "v": f32(np.append(target["right"], 0.5))})
        events.append({"ev": "left_position", "v": f32(np.append(target["left"], 0.5))})
        for s in ("right", "left"):
            events.append({"ev": f"trigger_{s}", "v": f32([np.clip(RNG.normal(0.5, 0.5), -0.2, 1.2)])[0]})
        for name in ("joystick_x_left", "joystick_y_left", "joystick_x_right", "joystick_y_right"):
            events.append({"ev": name, "v": f32([RNG.uniform(-1.3, 1.3)])[0]})
        for b in "abxy":
            if RNG.random() < 0.01:
                buttons[b] = not buttons[b]
            events.append({"ev": f"button_{b}", "v": buttons[b]})
        if t % 2 == 1:
            events.append({"ev": "cmd_timer"})
    return events


def compare(name, py, cpp):
    if len(py) != len(cpp):
        print(f"FAIL {name}: python published {len(py)} messages, C++ {len(cpp)}")
        return False
    worst, by_topic = 0.0, {}
    for i, ((pt, pn, pd), (ct, cn, cd)) in enumerate(zip(py, cpp)):
        if pt != ct or pn != cn or len(pd) != len(cd):
            print(f"FAIL {name}: message {i}: python {pt} {pn} vs C++ {ct} {cn}")
            return False
        worst = max(worst, float(np.max(np.abs(np.array(pd) - np.array(cd)))) if pd else 0.0)
        by_topic[pt] = by_topic.get(pt, 0) + 1
    ok = worst <= 1e-12
    print(f"{'PASS' if ok else 'FAIL'} {name}: {len(py)} messages identical in topic/names, "
          f"max value diff {worst:.1e}")
    for topic, count in sorted(by_topic.items()):
        print(f"      {count:6d}  {topic}")
    return ok


def main() -> int:
    global REPO, PRESETS
    record_dir = None
    if "--record" in sys.argv:
        i = sys.argv.index("--record")
        record_dir = sys.argv[i + 1]
        del sys.argv[i:i + 2]
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    binary = sys.argv[1]
    REPO = pathlib.Path(sys.argv[2]).resolve()
    PRESETS = REPO / "config" / "grasp_presets.yaml"
    events = make_events()
    ok = True
    scenarios = []
    for mode, rate in (("real", 0.0), ("real", 200.0), ("sim", 0.0)):
        py = python_run(events, mode, rate)
        cpp = cpp_run(binary, events, mode, rate)
        name = f"mode={mode} cmd_rate={rate:g}"
        ok &= compare(name, py, cpp)
        scenarios.append({"name": name, "args": ["--mode", mode, "--cmd-rate-hz", str(rate)],
                          "expected": [list(m) for m in py]})
    if record_dir is not None:
        path = pathlib.Path(record_dir) / "bridge.json.gz"
        golden = {"part": "bridge", "reference": "dora_ros2_bridge/main.py (python, LPF off)",
                  "tol": 1e-12, "records": events, "scenarios": scenarios}
        with gzip.open(path, "wt") as f:
            json.dump(golden, f, separators=(",", ":"))
        print(f"recorded {len(scenarios)} scenarios → {path}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
