// Port of the dora-to-ros2 event loop (src/dora_ros2_bridge/main.py `_run`):
// what gets commanded to the arms and the Revo2 hands, given the stream of
// IK joint targets, triggers, thumbsticks, buttons, grip flags and controller
// feedback. Framework-free — the ROS 2 node feeds the events in and implements
// BridgeOutputs to publish; tests/check_bridge.py feeds identical events to
// the python node and compares every published message.
//
// One handler per python Dora input, same semantics:
//   onButton        button_a/b/x/y      → /vr_buttons on change; grasp preset cycling
//   onTrigger       trigger_left/right  → ramp 4 fingers of BOTH hands; publish both (real)
//   onStick         joystick_x/y_*      → ramp that hand's thumb; publish it (real)
//   onArmPosition   left/right_position → real: grip hold / safety ramp / pass-through
//                                          sim: right publishes the merged /joint_command
//   onCommandTimer  (--cmd-rate-hz)     → real: publish the latest arm commands
//   setGripEngaged / setGripTimer        ← python SetBool services from the receiver
//   setPhysicalFeedback                  ← */controller_state feedback (real)
//
// Not ported (on purpose, for now): the low-pass filters (--lpf-arm-hz,
// --lpf-hand-hz, --cmd-lpf-hz). Commands go out unfiltered, i.e. exactly what
// python does with those options at 0.
#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>
#include <string>
#include <vector>

#include "openarm_ik/types.hpp"
#include "teleop_bridge/constants.hpp"

namespace teleop_bridge {

using openarm_ik::ArmSide;
using openarm_ik::JointVector;
using HandVector = Eigen::Matrix<double, kNumHandJoints, 1>;

enum class BridgeMode { kSim, kReal };
enum class VrButton : std::size_t { kA = 0, kB = 1, kX = 2, kY = 3 };
enum class StickAxis : std::size_t { kX = 0, kY = 1 };
const char* buttonName(VrButton button);

struct GraspPreset {
  std::string name;
  HandVector closed = HandVector::Zero();  // joint angles at trigger 1 / stick fully deflected
};

struct PresetCycle {
  VrButton button = VrButton::kX;
  std::vector<ArmSide> hands;      // which hands switch on a press
  std::vector<std::string> order;  // ≥ 2 names from the presets
};

struct GraspConfig {
  std::vector<GraspPreset> presets;  // file order; the first is the default
  std::optional<PresetCycle> cycle;
};
// Built-in config when no presets file is given: only "box" (= HAND_CLOSED).
GraspConfig defaultGraspConfig();
// Loads config/grasp_presets.yaml. nullopt (after logging why) on any error.
std::optional<GraspConfig> loadGraspConfig(const std::string& path);

struct BridgeConfig {
  BridgeMode mode = BridgeMode::kReal;
  GraspConfig grasp = defaultGraspConfig();
  // > 0: arm commands are only stored by onArmPosition and published by
  // onCommandTimer (python --cmd-rate-hz). 0: published on each IK event.
  double cmd_rate_hz = 0.0;
  // Per hand (indexed by sideIndex): negate a thumbstick axis before it drives
  // the thumb (x → thumb_metacarpal, y → thumb_proximal). All false and a zero
  // initial thumb = main.py in dora-openarm-ros2 git; the Jetson's main.py
  // negates left y, right x and right y and starts the thumb at [1.57, 0].
  std::array<bool, openarm_ik::kNumArms> invert_thumb_x{};
  std::array<bool, openarm_ik::kNumArms> invert_thumb_y{};
  // Thumb target [metacarpal, proximal] (rad) before the first stick event.
  Eigen::Vector2d initial_thumb_rad = Eigen::Vector2d::Zero();
};

// Joint names, same strings and order as main.py.
std::vector<std::string> armJointNames(ArmSide side);   // openarm_<side>_joint1..7
std::vector<std::string> handJointNames(ArmSide side);  // <side>_<finger>_joint ×6
// --mode sim /joint_command: arms interleaved (left1, right1, …), then left
// hand (6 driven + 5 distal mirroring hand[1:6]), then the right hand.
std::vector<std::string> simJointCommandNames();
std::vector<double> simJointCommandPositions(const JointVector& left_arm,
                                             const JointVector& right_arm,
                                             const HandVector& left_hand,
                                             const HandVector& right_hand);

// Where the bridge sends its commands (the ROS 2 node publishes them).
class BridgeOutputs {
 public:
  virtual ~BridgeOutputs() = default;
  virtual void publishButtons(const std::array<int, kNumButtons>& buttons) = 0;
  virtual void publishHand(ArmSide side, const HandVector& positions) = 0;    // real
  virtual void publishArm(ArmSide side, const JointVector& positions) = 0;    // real
  virtual void publishJointCommand(const JointVector& left_arm, const JointVector& right_arm,
                                   const HandVector& left_hand,
                                   const HandVector& right_hand) = 0;         // sim
};

class TeleopBridgeLogic {
 public:
  // `outputs` must outlive this object.
  TeleopBridgeLogic(BridgeConfig config, BridgeOutputs& outputs);

  void onButton(VrButton button, bool pressed);
  void onTrigger(ArmSide side, double raw);
  void onStick(ArmSide side, StickAxis axis, double value);
  void onArmPosition(ArmSide side, const JointVector& target);
  void onCommandTimer();

  void setGripEngaged(ArmSide side, bool engaged);
  void setGripTimer(ArmSide side, bool running);
  void setPhysicalFeedback(ArmSide side, const JointVector& positions);

  bool feedbackReady(ArmSide side) const { return arm(side).feedback_ready; }
  const std::string& handPreset(ArmSide side) const { return hand(side).preset; }
  const BridgeConfig& config() const { return config_; }

 private:
  struct ArmState {
    JointVector target = JointVector::Zero();           // latest IK target (cache[:7])
    JointVector internal_target = JointVector::Zero();  // safety-ramp state
    std::optional<JointVector> hold;                    // latched freeze pose
    std::optional<JointVector> stored_command;          // for the fixed-rate timer
    JointVector physical = JointVector::Zero();
    bool feedback_ready = false;
    bool grip_engaged = false;  // python default False → freeze until engaged
    bool grip_timer = false;
    std::uint64_t events = 0;
  };
  struct HandState {
    HandVector target = HandVector::Zero();
    double desired_grip = 0.0;
    std::array<double, 2> stick{};  // [x, y]
    std::string preset;
  };

  ArmState& arm(ArmSide side) { return arms_.at(openarm_ik::sideIndex(side)); }
  const ArmState& arm(ArmSide side) const { return arms_.at(openarm_ik::sideIndex(side)); }
  HandState& hand(ArmSide side) { return hands_.at(openarm_ik::sideIndex(side)); }
  const HandState& hand(ArmSide side) const { return hands_.at(openarm_ik::sideIndex(side)); }
  const HandVector& presetClosed(const std::string& name) const;

  JointVector safeArmCommand(ArmSide side);  // the real-mode grip logic
  void sendArm(ArmSide side, const JointVector& command);
  void rampFingers(ArmSide side);
  void rampThumb(ArmSide side);
  void cyclePresets();

  BridgeConfig config_;
  BridgeOutputs& outputs_;
  std::array<ArmState, openarm_ik::kNumArms> arms_{};
  std::array<HandState, openarm_ik::kNumArms> hands_{};
  std::array<int, kNumButtons> buttons_{};
};

}  // namespace teleop_bridge
