// openarm_teleop_node: the whole VR teleop dataflow in one ROS 2 process.
//
// Threads:
//   UDP       UdpJsonReceiver: recv() → keeps only the freshest Quest packet.
//   control   fixed-rate loop (tick_hz): QuestTeleopInput (smoothing, grip) →
//             IkSolver → TeleopBridgeLogic → publish. Owns the IK.
//   IK right  ┐ IkSolver workers (ik.threads_per_arm): one arm each, own MuJoCo
//   IK left   ┘ data, exchange the shared LM μ at a spin barrier per iteration.
//   ROS 2     executor (main thread): /joint_states, */controller_state,
//             the fixed-rate arm command timer.
// Shared between control and ROS 2: the bridge logic (bridge_mutex_) and the
// initial joint positions (init_mutex_). rclcpp publishers are thread-safe.
//
// Replaces the Dora nodes udp-receiver, ik, fk and dora-to-ros2 of
// config/dataflow_bridge_ros2_vr_real.yaml: same topics, messages and grip /
// safety-ramp / hand logic (tests/check_bridge.py); low-pass filters not
// ported yet. Differences on purpose:
//   * the IK starts from the robot's real joints (/joint_states) instead of
//     all zeros, so the first solve does not jump;
//   * grip flags go straight from the receiver logic to the bridge logic (in
//     python they travel over the /dora_bridge/set_grip_* SetBool services).
//
// Run: build/openarm_teleop_node --ros-args --params-file config/teleop_real.yaml
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unordered_map>

#include "openarm_ik/ik_solver.hpp"
#include "quest_vr/constants.hpp"
#include "quest_vr/logging.hpp"
#include "quest_vr/teleop_input.hpp"
#include "quest_vr/udp_receiver.hpp"
#include "teleop_bridge/bridge_logic.hpp"

namespace {

using openarm_ik::ArmSide;
using teleop_bridge::HandVector;
using teleop_bridge::JointVector;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// ── parameter defaults (= the dataflow yaml / python defaults) ──────────────
constexpr double kDefaultTickHz = 500.0;            // dora/timer/millis/2
constexpr double kDefaultControllerTiltDeg = 25.0;  // ik.py _Y_TILT_DEG
constexpr int kQueueDepth = 10;                     // python QoSProfile depth
constexpr double kMaxTickHz = 2000.0;
constexpr int kMaxPort = 65535;
constexpr auto kStatusPeriod = 1s;
constexpr int kWaitLogThrottleMs = 2000;
constexpr double kMicrosPerSecond = 1e6;
// Defaults for model_xml / grasp_presets, relative to this package's share dir.
constexpr const char* kPackageName = "openarm_teleop_cpp";
constexpr const char* kDefaultModelXmlRelative = "/config/openarm_mujoco/v1/scene.xml";
constexpr const char* kDefaultGraspPresetsRelative = "/config/grasp_presets.yaml";

const char* kTopicButtons = "/vr_buttons";
const char* kTopicJointCommand = "/joint_command";
const char* kTopicJointStates = "/joint_states";

// "/<side><suffix>", e.g. sideTopic(kRight, "_ee_pose") → "/right_ee_pose".
std::string sideTopic(ArmSide side, std::string_view suffix) {
  return std::format("/{}{}", openarm_ik::sideName(side), suffix);
}

trajectory_msgs::msg::JointTrajectory makeTrajectory(const std::vector<std::string>& names,
                                                     const double* data, std::size_t size) {
  // main.py _make_traj_msg: zero stamp, one point, zero time_from_start → execute now.
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(data, data + size);
  msg.points.push_back(point);
  return msg;
}

class TeleopNode : public rclcpp::Node, public teleop_bridge::BridgeOutputs {
 public:
  TeleopNode() : rclcpp::Node("openarm_teleop") {
    installLogSink();
    ok_ = configure();
  }
  ~TeleopNode() override {
    stop_.store(true);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    if (receiver_ != nullptr) {
      receiver_->stop();
    }
    quest_vr::setLogSink({});  // the sink captures this node's logger
  }
  bool ok() const { return ok_; }

  // ── BridgeOutputs (called with bridge_mutex_ held, any thread) ────────────
  void publishButtons(const std::array<int, teleop_bridge::kNumButtons>& buttons) override {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();
    msg.buttons.assign(buttons.begin(), buttons.end());
    buttons_pub_->publish(msg);
  }
  void publishHand(ArmSide side, const HandVector& positions) override {
    const std::size_t k = openarm_ik::sideIndex(side);
    hand_pub_.at(k)->publish(makeTrajectory(hand_names_.at(k), positions.data(),
                                            static_cast<std::size_t>(positions.size())));
  }
  void publishArm(ArmSide side, const JointVector& positions) override {
    const std::size_t k = openarm_ik::sideIndex(side);
    if (arm_pub_.at(k) == nullptr) {
      return;  // sim mode has no per-arm publisher (the bridge never calls this there)
    }
    arm_pub_.at(k)->publish(makeTrajectory(arm_names_.at(k), positions.data(),
                                           static_cast<std::size_t>(positions.size())));
    arm_commands_.fetch_add(1, std::memory_order_relaxed);
  }
  void publishJointCommand(const JointVector& left_arm, const JointVector& right_arm,
                           const HandVector& left_hand, const HandVector& right_hand) override {
    if (joint_command_pub_ == nullptr) {
      return;
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = sim_names_;
    msg.position =
        teleop_bridge::simJointCommandPositions(left_arm, right_arm, left_hand, right_hand);
    joint_command_pub_->publish(msg);
    arm_commands_.fetch_add(1, std::memory_order_relaxed);
  }

 private:
  struct Stats {  // control thread only
    std::uint64_t ticks = 0;
    std::uint64_t fresh_packets = 0;
    std::uint64_t solves = 0;
    double solve_time_sum_us = 0.0;
    double solve_time_max_us = 0.0;
    double tick_time_sum_us = 0.0;
    double tick_time_max_us = 0.0;
    std::uint64_t overruns = 0;  // ticks that started late by a full period
  };

  void installLogSink() {
    const rclcpp::Logger logger = get_logger();
    quest_vr::setLogSink([logger](quest_vr::LogLevel level, const std::string& message) {
      switch (level) {
        case quest_vr::LogLevel::kDebug:
          RCLCPP_DEBUG(logger, "%s", message.c_str());
          break;
        case quest_vr::LogLevel::kInfo:
          RCLCPP_INFO(logger, "%s", message.c_str());
          break;
        case quest_vr::LogLevel::kWarn:
          RCLCPP_WARN(logger, "%s", message.c_str());
          break;
        case quest_vr::LogLevel::kError:
          RCLCPP_ERROR(logger, "%s", message.c_str());
          break;
      }
    });
  }

  bool configure() {
    // ── parameters ──────────────────────────────────────────────────────────
    const auto host = declare_parameter<std::string>("host", std::string(quest_vr::kDefaultHost));
    const auto port = declare_parameter<std::int64_t>("port", quest_vr::kDefaultPort);
    tick_hz_ = declare_parameter<double>("tick_hz", kDefaultTickHz);
    const auto mode = declare_parameter<std::string>("mode", "real");
    const auto arms = declare_parameter<std::string>("arms", "bimanual");
    // Defaults: the copies installed in share/openarm_teleop_cpp/config.
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory(kPackageName);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
                  "package '%s' not found (%s): model_xml and grasp_presets have no default",
                  kPackageName, e.what());
    }
    const std::string default_model_xml =
        share_dir.empty() ? std::string() : share_dir + kDefaultModelXmlRelative;
    const std::string default_presets =
        share_dir.empty() ? std::string() : share_dir + kDefaultGraspPresetsRelative;
    const auto model_xml = declare_parameter<std::string>("model_xml", default_model_xml);
    const auto presets_path = declare_parameter<std::string>("grasp_presets", default_presets);
    const auto cmd_rate_hz = declare_parameter<double>("cmd_rate_hz", 0.0);
    const auto tilt_deg = declare_parameter<double>("controller_tilt_deg", kDefaultControllerTiltDeg);
    const auto reference_cache = declare_parameter<bool>("reference_cache", true);
    const bool wait_for_joint_states = declare_parameter<bool>("wait_for_joint_states", true);
    publish_ee_pose_ = declare_parameter<bool>("publish_ee_pose", true);
    openarm_ik::IkParams ik_params;
    ik_params.max_iterations = static_cast<int>(
        declare_parameter<std::int64_t>("ik.max_iterations", ik_params.max_iterations));
    ik_params.damping = declare_parameter<double>("ik.damping", ik_params.damping);
    ik_params.lm_damping = declare_parameter<double>("ik.lm_damping", ik_params.lm_damping);
    ik_params.posture_cost = declare_parameter<double>("ik.posture_cost", ik_params.posture_cost);
    ik_params.position_cost = declare_parameter<double>("ik.position_cost", ik_params.position_cost);
    ik_params.orientation_cost =
        declare_parameter<double>("ik.orientation_cost", ik_params.orientation_cost);
    const auto backend = declare_parameter<std::string>("ik.qp_backend", "daqp");
    const bool threads_per_arm = declare_parameter<bool>("ik.threads_per_arm", true);

    if (port <= 0 || port > kMaxPort) {
      RCLCPP_FATAL(get_logger(), "port %ld out of range 1..%d", static_cast<long>(port), kMaxPort);
      return false;
    }
    if (!(tick_hz_ > 0.0) || tick_hz_ > kMaxTickHz) {
      RCLCPP_FATAL(get_logger(), "tick_hz %.1f out of range (0, %.0f]", tick_hz_, kMaxTickHz);
      return false;
    }
    if (mode != "real" && mode != "sim") {
      RCLCPP_FATAL(get_logger(), "mode '%s' must be 'real' or 'sim'", mode.c_str());
      return false;
    }
    if (model_xml.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "parameter model_xml is empty: source the workspace install/setup.bash "
                   "or pass -p model_xml:=<scene.xml>");
      return false;
    }
    if (backend == "daqp") {
      ik_params.qp_backend = openarm_ik::QpBackend::kDaqp;
    } else if (backend == "activeset") {
      ik_params.qp_backend = openarm_ik::QpBackend::kActiveSet;
    } else {
      RCLCPP_FATAL(get_logger(), "ik.qp_backend '%s' must be daqp|activeset", backend.c_str());
      return false;
    }
    ik_params.threading = threads_per_arm ? openarm_ik::IkThreading::kThreadPerArm
                                          : openarm_ik::IkThreading::kSingleThread;
    std::vector<ArmSide> active;
    if (arms == "bimanual" || arms == "right") {
      active.push_back(ArmSide::kRight);
    }
    if (arms == "bimanual" || arms == "left") {
      active.push_back(ArmSide::kLeft);
    }
    if (active.empty()) {
      RCLCPP_FATAL(get_logger(), "arms '%s' must be bimanual|right|left", arms.c_str());
      return false;
    }
    if (!std::isfinite(tilt_deg)) {
      RCLCPP_FATAL(get_logger(), "controller_tilt_deg must be finite");
      return false;
    }
    // ik.py _apply_y_tilt: right-multiply → rotate about the controller's own Y.
    tilt_ = Eigen::Quaterniond(
        Eigen::AngleAxisd(tilt_deg * std::numbers::pi / 180.0, Eigen::Vector3d::UnitY()));

    // ── IK ──────────────────────────────────────────────────────────────────
    openarm_ik::ArmKinematics::Config model;
    model.xml_path = model_xml;
    ik_ = openarm_ik::IkSolver::create(model, ik_params, active);
    if (ik_ == nullptr) {
      return false;  // create() logged why
    }
    ik_started_ = !wait_for_joint_states;

    // ── bridge ──────────────────────────────────────────────────────────────
    teleop_bridge::BridgeConfig bridge_config;
    bridge_config.mode =
        mode == "sim" ? teleop_bridge::BridgeMode::kSim : teleop_bridge::BridgeMode::kReal;
    bridge_config.cmd_rate_hz = cmd_rate_hz;
    if (!presets_path.empty()) {
      const auto grasp = teleop_bridge::loadGraspConfig(presets_path);
      if (!grasp.has_value()) {
        return false;  // loadGraspConfig() logged why
      }
      bridge_config.grasp = *grasp;
    } else {
      RCLCPP_INFO(get_logger(), "no grasp_presets file: only the built-in 'box' hand pose");
    }
    bridge_ = std::make_unique<teleop_bridge::TeleopBridgeLogic>(bridge_config, *this);

    // ── ROS interfaces (same topics / QoS as main.py) ───────────────────────
    const auto qos_reliable = rclcpp::QoS(kQueueDepth).reliable();
    const auto qos_best_effort = rclcpp::QoS(kQueueDepth).best_effort();
    buttons_pub_ = create_publisher<sensor_msgs::msg::Joy>(kTopicButtons, qos_best_effort);
    for (const ArmSide side : openarm_ik::kArmSides) {
      const std::size_t k = openarm_ik::sideIndex(side);
      arm_names_.at(k) = teleop_bridge::armJointNames(side);
      hand_names_.at(k) = teleop_bridge::handJointNames(side);
      ee_pose_pub_.at(k) = create_publisher<geometry_msgs::msg::PoseStamped>(
          sideTopic(side, "_ee_pose"), qos_best_effort);
      hand_pub_.at(k) = create_publisher<trajectory_msgs::msg::JointTrajectory>(
          sideTopic(side, "_revo2_hand_controller/joint_trajectory"), qos_reliable);
      if (bridge_config.mode == teleop_bridge::BridgeMode::kReal) {
        arm_pub_.at(k) = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            sideTopic(side, "_joint_trajectory_controller/joint_trajectory"), qos_reliable);
        feedback_sub_.at(k) =
            create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
                sideTopic(side, "_joint_trajectory_controller/controller_state"), kQueueDepth,
                [this, side](const control_msgs::msg::JointTrajectoryControllerState& msg) {
                  onControllerState(side, msg);
                });
      }
    }
    if (bridge_config.mode == teleop_bridge::BridgeMode::kSim) {
      sim_names_ = teleop_bridge::simJointCommandNames();
      joint_command_pub_ =
          create_publisher<sensor_msgs::msg::JointState>(kTopicJointCommand, qos_reliable);
    }
    if (wait_for_joint_states) {
      joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          kTopicJointStates, kQueueDepth,
          [this](const sensor_msgs::msg::JointState& msg) { onJointStates(msg); });
    }
    if (bridge_config.cmd_rate_hz > 0.0 && bridge_config.mode == teleop_bridge::BridgeMode::kReal) {
      cmd_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / bridge_config.cmd_rate_hz),
                                     [this] {
                                       const std::lock_guard<std::mutex> lock(bridge_mutex_);
                                       bridge_->onCommandTimer();
                                     });
    }

    // ── receiver + control thread ───────────────────────────────────────────
    teleop_ = std::make_unique<quest_vr::QuestTeleopInput>(quest_vr::QuestPoseProcessor(
        reference_cache ? quest_vr::defaultReferenceCachePath() : std::nullopt));
    receiver_ = std::make_unique<quest_vr::UdpJsonReceiver>(host, static_cast<std::uint16_t>(port));
    if (!receiver_->start()) {
      return false;  // start() logged why
    }
    control_thread_ = std::thread([this] { controlLoop(); });
    RCLCPP_INFO(get_logger(),
                "ready: UDP %s:%ld, control %.0f Hz, mode %s, arms %s, IK %s, tilt %.1f deg, "
                "IK starts from %s",
                host.c_str(), static_cast<long>(port), tick_hz_, mode.c_str(), arms.c_str(),
                threads_per_arm ? "1 thread per arm" : "single thread", tilt_deg,
                wait_for_joint_states ? "/joint_states" : "model qpos0 (python behaviour)");
    return true;
  }

  // ── ROS 2 thread ──────────────────────────────────────────────────────────
  void onJointStates(const sensor_msgs::msg::JointState& msg) {
    {
      const std::lock_guard<std::mutex> lock(init_mutex_);
      if (initial_joints_.has_value()) {
        return;  // already captured; the control thread takes it on its next tick
      }
    }
    if (msg.name.size() != msg.position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs,
                           "/joint_states: %zu names but %zu positions, ignored", msg.name.size(),
                           msg.position.size());
      return;
    }
    std::unordered_map<std::string, double> by_name;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      by_name.emplace(msg.name[i], msg.position[i]);
    }
    std::array<JointVector, openarm_ik::kNumArms> q{};
    for (const ArmSide side : openarm_ik::kArmSides) {
      const std::size_t k = openarm_ik::sideIndex(side);
      for (std::size_t j = 0; j < arm_names_.at(k).size(); ++j) {
        const auto it = by_name.find(arm_names_.at(k)[j]);
        if (it == by_name.end()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs,
                               "/joint_states has no '%s' yet, IK still waiting",
                               arm_names_.at(k)[j].c_str());
          return;
        }
        q.at(k)(static_cast<Eigen::Index>(j)) = it->second;
      }
    }
    const std::lock_guard<std::mutex> lock(init_mutex_);
    initial_joints_ = q;
  }

  void onControllerState(ArmSide side,
                         const control_msgs::msg::JointTrajectoryControllerState& msg) {
    constexpr std::size_t kArmJoints = openarm_ik::kNumArmJoints;
    if (msg.feedback.positions.size() < kArmJoints) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs,
                           "%s controller_state has %zu positions (< %zu), ignored",
                           openarm_ik::sideName(side), msg.feedback.positions.size(), kArmJoints);
      return;
    }
    const std::lock_guard<std::mutex> lock(bridge_mutex_);
    bridge_->setPhysicalFeedback(side, Eigen::Map<const JointVector>(msg.feedback.positions.data()));
  }

  // ── control thread ────────────────────────────────────────────────────────
  void controlLoop() {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / tick_hz_));
    t0_ = Clock::now();
    auto next_tick = t0_;
    auto next_status = t0_ + kStatusPeriod;
    while (!stop_.load() && rclcpp::ok()) {
      next_tick += period;
      std::this_thread::sleep_until(next_tick);
      const auto tick_start = Clock::now();
      if (tick_start - next_tick > period) {
        ++stats_.overruns;
        next_tick = tick_start;  // do not try to catch up with a burst of ticks
      }
      tick();
      const double tick_us =
          std::chrono::duration<double>(Clock::now() - tick_start).count() * kMicrosPerSecond;
      stats_.tick_time_sum_us += tick_us;
      stats_.tick_time_max_us = std::max(stats_.tick_time_max_us, tick_us);
      if (Clock::now() >= next_status) {
        next_status += kStatusPeriod;
        logStatus();
      }
    }
  }

  void takeInitialJoints() {
    std::optional<std::array<JointVector, openarm_ik::kNumArms>> q;
    {
      const std::lock_guard<std::mutex> lock(init_mutex_);
      q = initial_joints_;
    }
    if (!q.has_value()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs,
                           "IK waiting for /joint_states before the first solve");
      return;
    }
    for (const ArmSide side : openarm_ik::kArmSides) {
      if (!ik_->syncJointPositions(side, q->at(openarm_ik::sideIndex(side)))) {
        return;  // logged; retried next tick
      }
    }
    ik_started_ = true;
    RCLCPP_INFO(get_logger(), "IK initialised from /joint_states (right j1..3 = %.3f %.3f %.3f)",
                (*q)[0](0), (*q)[0](1), (*q)[0](2));
  }

  void tick() {
    const auto received = receiver_->latest();
    if (!received.has_value()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs,
                           "waiting for the first Quest packet");
      return;  // python: `if msg is None: continue`
    }
    ++stats_.ticks;
    if (received->sequence != last_sequence_) {
      ++stats_.fresh_packets;
      last_sequence_ = received->sequence;
    }
    const double now_s = std::chrono::duration<double>(Clock::now() - t0_).count();
    last_frame_ = teleop_->step(received->packet, now_s);
    const quest_vr::TeleopFrame& frame = last_frame_;

    if (!ik_started_) {
      takeInitialJoints();
    }
    std::optional<std::array<JointVector, openarm_ik::kNumArms>> solved;
    if (ik_started_) {
      solved = solveIk(frame);  // outside bridge_mutex_: the ROS thread is never blocked by IK
    }

    const std::lock_guard<std::mutex> lock(bridge_mutex_);
    // Python pushes the grip flags (and waits for the ack) before the poses.
    for (const ArmSide side : openarm_ik::kArmSides) {
      const quest_vr::ArmFrame& arm = side == ArmSide::kRight ? frame.right : frame.left;
      bridge_->setGripEngaged(side, arm.engaged);
      bridge_->setGripTimer(side, arm.grip_timer_running);
    }
    if (solved.has_value()) {
      // Python ik sends position_right then position_left.
      bridge_->onArmPosition(ArmSide::kRight, (*solved)[openarm_ik::sideIndex(ArmSide::kRight)]);
      bridge_->onArmPosition(ArmSide::kLeft, (*solved)[openarm_ik::sideIndex(ArmSide::kLeft)]);
    }
    // Remaining receiver outputs, in python's send order.
    using teleop_bridge::StickAxis;
    using teleop_bridge::VrButton;
    if (frame.trigger_right) {
      bridge_->onTrigger(ArmSide::kRight, *frame.trigger_right);
    }
    if (frame.trigger_left) {
      bridge_->onTrigger(ArmSide::kLeft, *frame.trigger_left);
    }
    const std::array<std::tuple<const std::optional<double>*, ArmSide, StickAxis>, 4> sticks = {{
        {&frame.joystick_x_left, ArmSide::kLeft, StickAxis::kX},
        {&frame.joystick_y_left, ArmSide::kLeft, StickAxis::kY},
        {&frame.joystick_x_right, ArmSide::kRight, StickAxis::kX},
        {&frame.joystick_y_right, ArmSide::kRight, StickAxis::kY},
    }};
    for (const auto& [value, side, axis] : sticks) {
      if (value->has_value()) {
        bridge_->onStick(side, axis, **value);
      }
    }
    const std::array<std::pair<const std::optional<bool>*, VrButton>, 4> buttons = {{
        {&frame.button_a, VrButton::kA},
        {&frame.button_b, VrButton::kB},
        {&frame.button_x, VrButton::kX},
        {&frame.button_y, VrButton::kY},
    }};
    for (const auto& [value, button] : buttons) {
      if (value->has_value()) {
        bridge_->onButton(button, **value);
      }
    }
  }

  std::optional<std::array<JointVector, openarm_ik::kNumArms>> solveIk(
      const quest_vr::TeleopFrame& frame) {
    for (const ArmSide side : openarm_ik::kArmSides) {
      const quest_vr::ArmFrame& arm = side == ArmSide::kRight ? frame.right : frame.left;
      if (!arm.pose.has_value() || !ik_->isActive(side)) {
        continue;  // python: no pose_* output until the first engagement
      }
      openarm_ik::FramePose target;
      target.position = arm.pose->position;
      target.orientation = arm.pose->orientation * tilt_;
      ik_->setTarget(side, target);
    }
    if (!ik_->ready()) {
      return std::nullopt;  // bimanual: both arms need a target (python _pending)
    }
    const auto start = Clock::now();
    const openarm_ik::IkStatus status = ik_->solve();
    const double elapsed_us =
        std::chrono::duration<double>(Clock::now() - start).count() * kMicrosPerSecond;
    if (status != openarm_ik::IkStatus::kSolved) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWaitLogThrottleMs, "IK: %s",
                           openarm_ik::ikStatusName(status));
      return std::nullopt;
    }
    ++stats_.solves;
    stats_.solve_time_sum_us += elapsed_us;
    stats_.solve_time_max_us = std::max(stats_.solve_time_max_us, elapsed_us);
    if (publish_ee_pose_) {
      publishEePoses();
    }
    std::array<JointVector, openarm_ik::kNumArms> q{};
    for (const ArmSide side : openarm_ik::kArmSides) {
      q[openarm_ik::sideIndex(side)] = ik_->jointPositions(side);
    }
    return q;
  }

  void publishEePoses() {
    // Same as the python fk node: MuJoCo world frame, empty frame_id.
    for (const ArmSide side : openarm_ik::kArmSides) {
      const openarm_ik::FramePose pose = ik_->tcpPose(side);
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now();
      msg.pose.position.x = pose.position.x();
      msg.pose.position.y = pose.position.y();
      msg.pose.position.z = pose.position.z();
      msg.pose.orientation.w = pose.orientation.w();
      msg.pose.orientation.x = pose.orientation.x();
      msg.pose.orientation.y = pose.orientation.y();
      msg.pose.orientation.z = pose.orientation.z();
      ee_pose_pub_.at(openarm_ik::sideIndex(side))->publish(msg);
    }
  }

  void logStatus() {
    const auto mean = [](double sum, std::uint64_t n) {
      return n > 0 ? sum / static_cast<double>(n) : 0.0;
    };
    std::string presets;
    {
      const std::lock_guard<std::mutex> lock(bridge_mutex_);
      presets = std::format("R={} L={}", bridge_->handPreset(ArmSide::kRight),
                            bridge_->handPreset(ArmSide::kLeft));
    }
    RCLCPP_INFO(get_logger(),
                "%lu ticks/s (mean %.0f us, max %.0f us, %lu late) | %lu packets/s | %lu IK/s "
                "(mean %.0f us, max %.0f us) | %lu arm cmds/s | grip R=%d L=%d | preset %s",
                stats_.ticks, mean(stats_.tick_time_sum_us, stats_.ticks), stats_.tick_time_max_us,
                stats_.overruns, stats_.fresh_packets, stats_.solves,
                mean(stats_.solve_time_sum_us, stats_.solves), stats_.solve_time_max_us,
                arm_commands_.exchange(0), last_frame_.right.engaged, last_frame_.left.engaged,
                presets.c_str());
    stats_ = Stats{};
  }

  bool ok_ = false;
  double tick_hz_ = kDefaultTickHz;
  bool publish_ee_pose_ = true;
  Eigen::Quaterniond tilt_ = Eigen::Quaterniond::Identity();

  // control thread only
  std::unique_ptr<openarm_ik::IkSolver> ik_;
  std::unique_ptr<quest_vr::QuestTeleopInput> teleop_;
  bool ik_started_ = false;
  Clock::time_point t0_;
  std::uint64_t last_sequence_ = 0;
  quest_vr::TeleopFrame last_frame_;
  Stats stats_;

  // shared
  std::mutex bridge_mutex_;
  std::unique_ptr<teleop_bridge::TeleopBridgeLogic> bridge_;  // guarded by bridge_mutex_
  std::mutex init_mutex_;
  std::optional<std::array<JointVector, openarm_ik::kNumArms>> initial_joints_;
  std::atomic<std::uint64_t> arm_commands_{0};
  std::atomic<bool> stop_{false};

  std::unique_ptr<quest_vr::UdpJsonReceiver> receiver_;
  std::thread control_thread_;

  std::array<std::vector<std::string>, openarm_ik::kNumArms> arm_names_;
  std::array<std::vector<std::string>, openarm_ik::kNumArms> hand_names_;
  std::vector<std::string> sim_names_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  std::array<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr,
             openarm_ik::kNumArms>
      arm_pub_{};
  std::array<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr,
             openarm_ik::kNumArms>
      hand_pub_{};
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, openarm_ik::kNumArms>
      ee_pose_pub_{};
  std::array<rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr,
             openarm_ik::kNumArms>
      feedback_sub_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = EXIT_SUCCESS;
  {
    const auto node = std::make_shared<TeleopNode>();
    if (node->ok()) {
      rclcpp::spin(node);  // ROS 2 thread = main thread
    } else {
      RCLCPP_FATAL(node->get_logger(), "configuration failed, exiting");
      exit_code = EXIT_FAILURE;
    }
  }
  rclcpp::shutdown();
  return exit_code;
}
