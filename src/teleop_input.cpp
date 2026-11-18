#include "quest_vr/teleop_input.hpp"

#include <cmath>

#include "quest_vr/constants.hpp"
#include "quest_vr/logging.hpp"

namespace quest_vr {

QuestTeleopInput::QuestTeleopInput(QuestPoseProcessor pose_processor,
                                   const OneEuroParams& smoother_params)
    : pose_processor_(std::move(pose_processor)),
      right_("RIGHT", smoother_params),
      left_("LEFT", smoother_params) {}

TeleopFrame QuestTeleopInput::step(const QuestPacket& packet, double now_s) {
  TeleopFrame frame;
  if (!std::isfinite(now_s)) {
    // Every timer/smoother below depends on now_s; refuse rather than corrupt state.
    logf(LogLevel::kError, "step(): non-finite now_s, skipping tick");
    return frame;
  }

  const Validity overall = packet.validity.value_or(Validity::kOk);
  const Validity right_validity = packet.right_validity.value_or(Validity::kOk);
  const Validity left_validity = packet.left_validity.value_or(Validity::kOk);
  if (overall != prev_overall_validity_) {
    logf(LogLevel::kInfo, "validity: {} -> {} (L={}, R={})", validityName(prev_overall_validity_),
         validityName(overall), validityName(left_validity), validityName(right_validity));
    prev_overall_validity_ = overall;
  }
  frame.overall_validity = overall;

  const QuestPoseProcessor::Result raw = pose_processor_.process(packet);

  frame.right = stepArm(right_, raw.right, packet.right_grip.value_or(0.0), right_validity, now_s);
  frame.left = stepArm(left_, raw.left, packet.left_grip.value_or(0.0), left_validity, now_s);

  // Buttons / triggers / sticks are forwarded regardless of pose validity.
  frame.trigger_right = packet.right_trigger;
  frame.trigger_left = packet.left_trigger;
  frame.joystick_y = packet.left_stick_y;
  frame.joystick_x_left = packet.left_stick_x;
  frame.joystick_y_left = packet.left_stick_y;
  frame.joystick_x_right = packet.right_stick_x;
  frame.joystick_y_right = packet.right_stick_y;
  frame.button_a = packet.button_a;
  frame.button_b = packet.button_b;
  frame.button_x = packet.button_x;
  frame.button_y = packet.button_y;
  return frame;
}

ArmFrame QuestTeleopInput::stepArm(ArmState& arm, const std::optional<Pose>& raw_pose,
                                   double grip, Validity validity, double now_s) {
  ArmFrame out;
  out.validity = validity;

  // ── Dead-man grip ──────────────────────────────────────────────────────────
  const bool engaged = grip > kGripEngageThreshold;
  const bool rising = engaged && !arm.prev_engaged;
  if (engaged != arm.prev_engaged) {
    logf(LogLevel::kInfo, "{} grip {} (grip={:.2f})", arm.name, engaged ? "engaged" : "released",
         grip);
  }
  if (rising) {
    // While released the smoother kept converging to wherever the controller
    // rested; reset so the first engaged pose is not dragged from there.
    arm.smoother.reset();
  }

  // ── Grip-hold timer: one-shot per fresh press, release does not cancel it ─
  if (rising) {
    arm.hold_start_s = now_s;
    arm.timer_fired = false;
    logf(LogLevel::kInfo, "{} grip -> timer START ({:.0f}s)", arm.name, kGripHoldWindowS);
  } else if (!arm.timer_fired && arm.hold_start_s.has_value() &&
             now_s - *arm.hold_start_s >= kGripHoldWindowS) {
    arm.timer_fired = true;
    logf(LogLevel::kInfo, "{} grip held {:.0f}s -> timer off", arm.name, kGripHoldWindowS);
  }
  out.engaged = engaged;
  out.grip_timer_running = arm.hold_start_s.has_value() && !arm.timer_fired;

  // ── Validity + smoothing ───────────────────────────────────────────────────
  std::optional<Pose> smoothed;
  if (validity == Validity::kInvalid) {
    if (arm.prev_validity != Validity::kInvalid) {
      logf(LogLevel::kWarn, "{} controller INVALID: pose output frozen, smoother reset",
           arm.name);
      arm.smoother.reset();
    }
  } else if (raw_pose.has_value()) {
    if (arm.prev_validity == Validity::kInvalid) {
      logf(LogLevel::kInfo, "{} controller valid again ({})", arm.name, validityName(validity));
    }
    // Smoothed every tick regardless of grip (python does the same).
    smoothed = arm.smoother.smooth(now_s, *raw_pose);
  }
  arm.prev_validity = validity;
  arm.prev_engaged = engaged;

  // ── Hold the last engaged pose while released ──────────────────────────────
  if (engaged && smoothed.has_value()) {
    if (!arm.last_sent_pose.has_value()) {
      logf(LogLevel::kInfo, "{} first engaged pose, starting pose output", arm.name);
    }
    arm.last_sent_pose = smoothed;
  }
  out.pose = arm.last_sent_pose;
  return out;
}

}  // namespace quest_vr
