// Port of the per-tick logic in quest_receiver.py `_run()` loop: pose
// processing, validity handling, One Euro smoothing, the dead-man grip and the
// 5 s grip-hold timer. Framework-free: the caller feeds the latest packet each
// tick and forwards the resulting frame (to Dora, ROS 2, the IK thread, ...).
//
// Not ported here: the ROS 2 SetBool service calls that push the grip
// engaged/timer flags to dora-to-ros2. The frame exposes those flags so the
// integration layer can publish them however it wants.
#pragma once

#include <optional>

#include "quest_vr/one_euro_smoother.hpp"
#include "quest_vr/packet.hpp"
#include "quest_vr/pose_processor.hpp"
#include "quest_vr/types.hpp"

namespace quest_vr {

struct ArmFrame {
  // Pose to send this tick. Live smoothed pose while the grip is engaged; the
  // last engaged pose (held) once released; nullopt until the first engagement.
  std::optional<Pose> pose;
  bool engaged = false;             // grip > threshold
  bool grip_timer_running = false;  // within kGripHoldWindowS of the last fresh press
  Validity validity = Validity::kOk;
};

struct TeleopFrame {
  Validity overall_validity = Validity::kOk;
  ArmFrame right;
  ArmFrame left;

  std::optional<double> trigger_right;
  std::optional<double> trigger_left;
  std::optional<double> joystick_y;  // legacy output: same value as joystick_y_left
  std::optional<double> joystick_x_left;
  std::optional<double> joystick_y_left;
  std::optional<double> joystick_x_right;
  std::optional<double> joystick_y_right;
  std::optional<bool> button_a;
  std::optional<bool> button_b;
  std::optional<bool> button_x;
  std::optional<bool> button_y;
};

class QuestTeleopInput {
 public:
  QuestTeleopInput(QuestPoseProcessor pose_processor, const OneEuroParams& smoother_params = {});

  // Process one tick. now_s: monotonic time in seconds (python: perf_counter()).
  TeleopFrame step(const QuestPacket& packet, double now_s);

 private:
  struct ArmState {
    explicit ArmState(const char* label, const OneEuroParams& params)
        : name(label), smoother(params) {}

    const char* name;
    OneEuroPoseSmoother smoother;
    Validity prev_validity = Validity::kOk;
    bool prev_engaged = false;
    std::optional<double> hold_start_s;
    bool timer_fired = false;
    std::optional<Pose> last_sent_pose;
  };

  static ArmFrame stepArm(ArmState& arm, const std::optional<Pose>& raw_pose, double grip,
                          Validity validity, double now_s);

  QuestPoseProcessor pose_processor_;
  ArmState right_;
  ArmState left_;
  Validity prev_overall_validity_ = Validity::kOk;
};

}  // namespace quest_vr
