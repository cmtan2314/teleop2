#include "teleop_bridge/bridge_logic.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "quest_vr/logging.hpp"

namespace teleop_bridge {
namespace {

using quest_vr::LogLevel;
using quest_vr::logf;

constexpr Eigen::Index kFingerStart = static_cast<Eigen::Index>(kNumThumbJoints);
constexpr Eigen::Index kNumFingers = static_cast<Eigen::Index>(kNumHandJoints - kNumThumbJoints);
constexpr Eigen::Index kNumThumb = static_cast<Eigen::Index>(kNumThumbJoints);

template <typename Derived>
auto clampStep(const Eigen::MatrixBase<Derived>& delta, double max_step) {
  return delta.cwiseMax(-max_step).cwiseMin(max_step);
}

}  // namespace

std::vector<std::string> armJointNames(ArmSide side) {
  std::vector<std::string> names;
  for (std::size_t i = 0; i < openarm_ik::kNumArmJoints; ++i) {
    names.push_back(std::format("openarm_{}_joint{}", openarm_ik::sideName(side), i + 1));
  }
  return names;
}

std::vector<std::string> handJointNames(ArmSide side) {
  std::vector<std::string> names;
  for (const std::string_view finger : kHandFingerNames) {
    names.push_back(std::format("{}_{}_joint", openarm_ik::sideName(side), finger));
  }
  return names;
}

std::vector<std::string> simJointCommandNames() {
  std::vector<std::string> names;
  const auto left = armJointNames(ArmSide::kLeft);
  const auto right = armJointNames(ArmSide::kRight);
  for (std::size_t i = 0; i < openarm_ik::kNumArmJoints; ++i) {
    names.push_back(left[i]);
    names.push_back(right[i]);
  }
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    for (const std::string& name : handJointNames(side)) {
      names.push_back(name);
    }
    for (const std::string_view distal : kHandDistalNames) {
      names.push_back(std::format("{}_{}_joint", openarm_ik::sideName(side), distal));
    }
  }
  return names;
}

std::vector<double> simJointCommandPositions(const JointVector& left_arm,
                                             const JointVector& right_arm,
                                             const HandVector& left_hand,
                                             const HandVector& right_hand) {
  std::vector<double> positions;
  for (Eigen::Index i = 0; i < left_arm.size(); ++i) {
    positions.push_back(left_arm(i));
    positions.push_back(right_arm(i));
  }
  for (const HandVector* h : {&left_hand, &right_hand}) {
    for (Eigen::Index i = 0; i < h->size(); ++i) {
      positions.push_back((*h)(i));
    }
    for (Eigen::Index i = 1; i < h->size(); ++i) {  // distal mirrors hand[1:6]
      positions.push_back((*h)(i));
    }
  }
  return positions;
}

const char* buttonName(VrButton button) {
  switch (button) {
    case VrButton::kA:
      return "a";
    case VrButton::kB:
      return "b";
    case VrButton::kX:
      return "x";
    case VrButton::kY:
      return "y";
  }
  return "?";
}

TeleopBridgeLogic::TeleopBridgeLogic(BridgeConfig config, BridgeOutputs& outputs)
    : config_(std::move(config)), outputs_(outputs) {
  if (config_.grasp.presets.empty()) {
    logf(LogLevel::kWarn, "bridge: empty grasp preset list, using the built-in '{}'",
         kDefaultPresetName);
    config_.grasp = defaultGraspConfig();
  }
  // Both hands start on the first preset, or on cycle.order[0] (python).
  const std::string& start = config_.grasp.cycle.has_value() ? config_.grasp.cycle->order.front()
                                                             : config_.grasp.presets.front().name;
  for (HandState& h : hands_) {
    h.preset = start;
    h.target.head(kNumThumb) = config_.initial_thumb_rad;  // python MOCK_THUMB
  }
  logf(LogLevel::kInfo, "bridge: mode={} cmd_rate={} ({}), hands start on preset '{}'{}",
       config_.mode == BridgeMode::kReal ? "real" : "sim", config_.cmd_rate_hz,
       config_.cmd_rate_hz > 0.0 ? "fixed-rate timer" : "per IK event", start,
       config_.grasp.cycle.has_value()
           ? std::format(", button {} cycles presets", buttonName(config_.grasp.cycle->button))
           : std::string());
}

const HandVector& TeleopBridgeLogic::presetClosed(const std::string& name) const {
  for (const GraspPreset& preset : config_.grasp.presets) {
    if (preset.name == name) {
      return preset.closed;
    }
  }
  // Unreachable: loadGraspConfig() validates cycle.order against the presets.
  logf(LogLevel::kError, "bridge: unknown preset '{}', using '{}'", name,
       config_.grasp.presets.front().name);
  return config_.grasp.presets.front().closed;
}

// ── buttons ─────────────────────────────────────────────────────────────────

void TeleopBridgeLogic::onButton(VrButton button, bool pressed) {
  const auto index = static_cast<std::size_t>(button);
  const int value = pressed ? 1 : 0;
  if (buttons_.at(index) == value) {
    return;  // unchanged: python publishes /vr_buttons only on change
  }
  buttons_.at(index) = value;
  outputs_.publishButtons(buttons_);

  if (config_.grasp.cycle.has_value() && button == config_.grasp.cycle->button && value == 1) {
    cyclePresets();
  }
}

void TeleopBridgeLogic::cyclePresets() {
  const PresetCycle& cycle = *config_.grasp.cycle;
  for (const ArmSide side : cycle.hands) {
    std::string& preset = hand(side).preset;
    const auto it = std::find(cycle.order.begin(), cycle.order.end(), preset);
    // Not in the order (python: index -1) → next is order[0].
    const std::size_t next =
        it == cycle.order.end()
            ? 0
            : (static_cast<std::size_t>(it - cycle.order.begin()) + 1) % cycle.order.size();
    preset = cycle.order.at(next);
  }
  logf(LogLevel::kInfo, "grasp preset: left '{}', right '{}'", hand(ArmSide::kLeft).preset,
       hand(ArmSide::kRight).preset);
}

// ── Revo2 hands ─────────────────────────────────────────────────────────────

void TeleopBridgeLogic::rampFingers(ArmSide side) {
  HandState& h = hand(side);
  const auto desired = h.desired_grip * presetClosed(h.preset).segment(kFingerStart, kNumFingers);
  auto fingers = h.target.segment(kFingerStart, kNumFingers);
  fingers += clampStep(desired - fingers, kHandMaxStepRad);
}

void TeleopBridgeLogic::rampThumb(ArmSide side) {
  HandState& h = hand(side);
  const std::size_t k = openarm_ik::sideIndex(side);
  const double stick_x = config_.invert_thumb_x.at(k) ? -h.stick[0] : h.stick[0];
  const double stick_y = config_.invert_thumb_y.at(k) ? -h.stick[1] : h.stick[1];
  Eigen::Vector2d deflection(std::clamp(stick_x, 0.0, kStickMax),
                             std::clamp(stick_y, 0.0, kStickMax));
  const Eigen::Vector2d desired =
      deflection.cwiseProduct(presetClosed(h.preset).head<kNumThumbJoints>());
  auto thumb = h.target.head(kNumThumb);
  thumb += clampStep(desired - thumb, kHandMaxStepRad);
}

void TeleopBridgeLogic::onTrigger(ArmSide side, double raw) {
  if (!std::isfinite(raw)) {
    logf(LogLevel::kWarn, "bridge: non-finite {} trigger ignored", openarm_ik::sideName(side));
    return;
  }
  const double clipped = std::clamp(raw, kTriggerMin, kTriggerMax);
  hand(side).desired_grip = std::pow(clipped, kGripLinearizeExponent);
  // Python ramps (and in real mode publishes) BOTH hands on either trigger.
  rampFingers(ArmSide::kLeft);
  rampFingers(ArmSide::kRight);
  if (config_.mode == BridgeMode::kReal) {
    outputs_.publishHand(ArmSide::kLeft, hand(ArmSide::kLeft).target);
    outputs_.publishHand(ArmSide::kRight, hand(ArmSide::kRight).target);
  }
}

void TeleopBridgeLogic::onStick(ArmSide side, StickAxis axis, double value) {
  if (!std::isfinite(value)) {
    logf(LogLevel::kWarn, "bridge: non-finite {} thumbstick ignored", openarm_ik::sideName(side));
    return;
  }
  hand(side).stick.at(static_cast<std::size_t>(axis)) = std::clamp(value, kStickMin, kStickMax);
  rampThumb(side);
  if (config_.mode == BridgeMode::kReal) {
    outputs_.publishHand(side, hand(side).target);
  }
}

// ── arms ────────────────────────────────────────────────────────────────────

void TeleopBridgeLogic::setGripEngaged(ArmSide side, bool engaged) {
  ArmState& a = arm(side);
  if (a.grip_engaged != engaged) {
    logf(LogLevel::kInfo, "bridge: {} grip {}", openarm_ik::sideName(side),
         engaged ? "engaged" : "released → hold");
  }
  a.grip_engaged = engaged;
}

void TeleopBridgeLogic::setGripTimer(ArmSide side, bool running) {
  ArmState& a = arm(side);
  if (a.grip_timer != running) {
    logf(LogLevel::kInfo, "bridge: {} grip window {}", openarm_ik::sideName(side),
         running ? "open → safety ramp" : "closed → direct tracking");
  }
  a.grip_timer = running;
}

void TeleopBridgeLogic::setPhysicalFeedback(ArmSide side, const JointVector& positions) {
  if (!positions.allFinite()) {
    logf(LogLevel::kWarn, "bridge: non-finite {} controller feedback ignored",
         openarm_ik::sideName(side));
    return;
  }
  ArmState& a = arm(side);
  if (!a.feedback_ready) {
    logf(LogLevel::kInfo, "bridge: first {} controller_state feedback received",
         openarm_ik::sideName(side));
  }
  a.physical = positions;
  a.feedback_ready = true;
}

JointVector TeleopBridgeLogic::safeArmCommand(ArmSide side) {
  ArmState& a = arm(side);
  if (!a.grip_engaged) {
    // Grip released → freeze: latch the physical pose at release and hold it.
    if (!a.hold.has_value()) {
      a.hold = a.feedback_ready ? a.physical : a.target;
      a.internal_target = *a.hold;
    }
    return *a.hold;
  }
  a.hold.reset();
  if (!a.grip_timer) {
    return a.target;  // window closed → full tracking
  }
  // Grip window open → get_safe_position: ramp by at most kArmMaxStepRad.
  if (!a.feedback_ready) {
    return a.internal_target;  // python returns the (unchanged) internal target
  }
  const double drift = (a.internal_target - a.physical).cwiseAbs().maxCoeff();
  if (drift > kArmSyncThresholdRad) {
    a.internal_target = a.physical;
  }
  a.internal_target += clampStep(a.target - a.internal_target, kArmMaxStepRad);
  return a.internal_target;
}

void TeleopBridgeLogic::sendArm(ArmSide side, const JointVector& command) {
  if (config_.cmd_rate_hz > 0.0) {
    arm(side).stored_command = command;  // published by onCommandTimer
    return;
  }
  outputs_.publishArm(side, command);
}

void TeleopBridgeLogic::onArmPosition(ArmSide side, const JointVector& target) {
  if (!target.allFinite()) {
    logf(LogLevel::kError, "bridge: non-finite {} arm target dropped", openarm_ik::sideName(side));
    return;
  }
  ArmState& a = arm(side);
  ++a.events;
  a.target = target;

  if (config_.mode == BridgeMode::kSim) {
    // Left only updates its cache; each right event publishes the merged state.
    if (side == ArmSide::kRight) {
      outputs_.publishJointCommand(arm(ArmSide::kLeft).target, a.target,
                                   hand(ArmSide::kLeft).target, hand(ArmSide::kRight).target);
    }
  } else {
    sendArm(side, safeArmCommand(side));
  }

  if (side == ArmSide::kRight && a.events % kHeartbeatEvery == 0) {
    logf(LogLevel::kInfo,
         "bridge: right #{} left #{} | feedback R={} L={} | engaged R={} L={} | window R={} L={}",
         a.events, arm(ArmSide::kLeft).events, a.feedback_ready,
         arm(ArmSide::kLeft).feedback_ready, a.grip_engaged, arm(ArmSide::kLeft).grip_engaged,
         a.grip_timer, arm(ArmSide::kLeft).grip_timer);
  }
}

void TeleopBridgeLogic::onCommandTimer() {
  // Python _publish_arms order: left, then right; nothing until the first command.
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmState& a = arm(side);
    if (a.stored_command.has_value()) {
      outputs_.publishArm(side, *a.stored_command);
    }
  }
}

}  // namespace teleop_bridge
