// Test harness for teleop_bridge: replays dora-to-ros2 input events (JSON
// lines on stdin) through TeleopBridgeLogic and prints every message it would
// publish (JSON lines on stdout), for tests/check_bridge.py.
//
// Usage: bridge_replay [--mode real|sim] [--grasp-presets FILE] [--cmd-rate-hz HZ]
// Input:  {"ev": "<python input id>", "v": <value>}
//   button_a|b|x|y (bool), trigger_left|right (float), joystick_{x,y}_{left,right}
//   (float), left_position|right_position ([8]), cmd_timer, feedback_left|right
//   ([7]), grip_engaged_left|right (bool), grip_timer_left|right (bool)
// Output: {"topic": "...", "names": [...], "data": [...]}
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <span>
#include <string>
#include <string_view>

#include "quest_vr/logging.hpp"
#include "teleop_bridge/bridge_logic.hpp"

namespace {

using Json = nlohmann::json;
using openarm_ik::ArmSide;
using quest_vr::LogLevel;
using quest_vr::logf;
using teleop_bridge::HandVector;
using teleop_bridge::JointVector;

template <typename Vec>
std::vector<double> toVector(const Vec& v) {
  return {v.data(), v.data() + v.size()};
}

class JsonOutputs : public teleop_bridge::BridgeOutputs {
 public:
  void publishButtons(const std::array<int, teleop_bridge::kNumButtons>& buttons) override {
    emit("/vr_buttons", {}, std::vector<double>(buttons.begin(), buttons.end()));
  }
  void publishHand(ArmSide side, const HandVector& positions) override {
    emit(std::format("/{}_revo2_hand_controller/joint_trajectory", openarm_ik::sideName(side)),
         teleop_bridge::handJointNames(side), toVector(positions));
  }
  void publishArm(ArmSide side, const JointVector& positions) override {
    emit(std::format("/{}_joint_trajectory_controller/joint_trajectory",
                     openarm_ik::sideName(side)),
         teleop_bridge::armJointNames(side), toVector(positions));
  }
  void publishJointCommand(const JointVector& left_arm, const JointVector& right_arm,
                           const HandVector& left_hand, const HandVector& right_hand) override {
    emit("/joint_command", teleop_bridge::simJointCommandNames(),
         teleop_bridge::simJointCommandPositions(left_arm, right_arm, left_hand, right_hand));
  }

 private:
  static void emit(const std::string& topic, const std::vector<std::string>& names,
                   const std::vector<double>& data) {
    std::cout << Json{{"topic", topic}, {"names", names}, {"data", data}}.dump() << '\n';
  }
};

bool readVector(const Json& value, Eigen::Index size, JointVector& out) {
  if (!value.is_array() || static_cast<Eigen::Index>(value.size()) < size) {
    logf(LogLevel::kError, "expected an array of at least {} numbers", size);
    return false;
  }
  for (Eigen::Index i = 0; i < size; ++i) {
    out(i) = value[static_cast<std::size_t>(i)].get<double>();
  }
  return true;
}

bool handleEvent(teleop_bridge::TeleopBridgeLogic& bridge, const std::string& ev,
                 const Json& v) {
  using teleop_bridge::StickAxis;
  using teleop_bridge::VrButton;
  constexpr auto kArm = static_cast<Eigen::Index>(openarm_ik::kNumArmJoints);
  const std::array<std::pair<std::string_view, VrButton>, 4> buttons = {
      {{"button_a", VrButton::kA}, {"button_b", VrButton::kB}, {"button_x", VrButton::kX},
       {"button_y", VrButton::kY}}};
  for (const auto& [name, button] : buttons) {
    if (ev == name) {
      bridge.onButton(button, v.get<bool>());
      return true;
    }
  }
  for (const ArmSide side : openarm_ik::kArmSides) {
    const std::string s = openarm_ik::sideName(side);
    JointVector q;
    if (ev == "trigger_" + s) {
      bridge.onTrigger(side, v.get<double>());
    } else if (ev == "joystick_x_" + s) {
      bridge.onStick(side, StickAxis::kX, v.get<double>());
    } else if (ev == "joystick_y_" + s) {
      bridge.onStick(side, StickAxis::kY, v.get<double>());
    } else if (ev == s + "_position") {
      if (!readVector(v, kArm, q)) {
        return false;
      }
      bridge.onArmPosition(side, q);
    } else if (ev == "feedback_" + s) {
      if (!readVector(v, kArm, q)) {
        return false;
      }
      bridge.setPhysicalFeedback(side, q);
    } else if (ev == "grip_engaged_" + s) {
      bridge.setGripEngaged(side, v.get<bool>());
    } else if (ev == "grip_timer_" + s) {
      bridge.setGripTimer(side, v.get<bool>());
    } else {
      continue;
    }
    return true;
  }
  if (ev == "cmd_timer") {
    bridge.onCommandTimer();
    return true;
  }
  logf(LogLevel::kError, "unknown event '{}'", ev);
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  quest_vr::setMinLogLevel(LogLevel::kWarn);
  const std::span<char*> args(argv, static_cast<std::size_t>(argc));
  teleop_bridge::BridgeConfig config;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string_view arg = args[i];
    const bool has_value = i + 1 < args.size();
    if (arg == "--mode" && has_value) {
      const std::string_view mode = args[++i];
      config.mode = mode == "sim" ? teleop_bridge::BridgeMode::kSim : teleop_bridge::BridgeMode::kReal;
    } else if (arg == "--grasp-presets" && has_value) {
      const auto grasp = teleop_bridge::loadGraspConfig(args[++i]);
      if (!grasp.has_value()) {
        return EXIT_FAILURE;  // loadGraspConfig() logged why
      }
      config.grasp = *grasp;
    } else if (arg == "--cmd-rate-hz" && has_value) {
      config.cmd_rate_hz = std::stod(args[++i]);
    } else {
      logf(LogLevel::kError, "unknown or incomplete argument '{}'", arg);
      return EXIT_FAILURE;
    }
  }
  JsonOutputs outputs;
  teleop_bridge::TeleopBridgeLogic bridge(config, outputs);

  std::string line;
  while (std::getline(std::cin, line)) {
    const Json record = Json::parse(line, nullptr, false);
    if (record.is_discarded() || !record.contains("ev")) {
      logf(LogLevel::kError, "bad input line: {}", line);
      return EXIT_FAILURE;
    }
    if (!handleEvent(bridge, record["ev"].get<std::string>(),
                     record.contains("v") ? record["v"] : Json())) {
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
