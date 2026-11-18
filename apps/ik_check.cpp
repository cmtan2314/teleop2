// Test harness for IK steps 2–4: reads JSON lines on stdin, writes JSON lines on
// stdout, so tests/check_ik.py can compare against mink / openarm_control 0.1.0.
//
//   ik_check se3              {"a": [wxyz_xyz], "b": [wxyz_xyz]}
//                          → {"log_a": [6], "rminus": [6], "inv_mul": [wxyz_xyz], "jlog_a": [36]}
//   ik_check boxqp            {"H": [49 row-major], "c": [7], "lo": [7], "hi": [7]}
//                          → {"x": [7], "status": "...", "iterations": n}
//   ik_check replay <scene.xml>
//                             {"right": [px,py,pz,qw,qx,qy,qz], "left": [...]}  (either optional)
//                          → {"status": "...", "right": [7 joints], "left": [7 joints]}
//     The replay runs bimanual with the default IkParams (= the dataflow yaml)
//     and solves only when ready(), exactly like the python node.
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "openarm_ik/arm_kinematics.hpp"
#include "openarm_ik/box_qp.hpp"
#include "openarm_ik/ik_solver.hpp"
#include "openarm_ik/se3.hpp"
#include "quest_vr/logging.hpp"

namespace {

using Json = nlohmann::json;
using openarm_ik::ArmSide;
using openarm_ik::FramePose;
using quest_vr::LogLevel;
using quest_vr::logf;

constexpr std::size_t kWxyzXyzSize = 7;  // mink layout [qw, qx, qy, qz, x, y, z]
constexpr std::size_t kPoseSize = 7;     // node layout  [px, py, pz, qw, qx, qy, qz]
constexpr std::size_t kQuatOffsetInPose = 3;
constexpr std::size_t kPosOffsetInWxyzXyz = 4;

bool readArray(const Json& record, const char* key, std::size_t size, std::vector<double>& out) {
  if (!record.contains(key) || !record[key].is_array() || record[key].size() != size) {
    logf(LogLevel::kError, "input record needs '{}': array of {} numbers", key, size);
    return false;
  }
  out = record[key].get<std::vector<double>>();
  return true;
}

FramePose fromWxyzXyz(const std::vector<double>& v) {
  FramePose pose;
  pose.orientation = Eigen::Quaterniond(v[0], v[1], v[2], v[3]);
  const std::size_t p = kPosOffsetInWxyzXyz;
  pose.position = Eigen::Vector3d(v[p], v[p + 1], v[p + 2]);
  return pose;
}

FramePose fromNodePose(const std::vector<double>& v) {
  FramePose pose;
  pose.position = Eigen::Vector3d(v[0], v[1], v[2]);
  const std::size_t q = kQuatOffsetInPose;
  pose.orientation = Eigen::Quaterniond(v[q], v[q + 1], v[q + 2], v[q + 3]);
  return pose;
}

template <typename Derived>
std::vector<double> rowMajor(const Eigen::MatrixBase<Derived>& m) {
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(m.size()));
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      out.push_back(m(r, c));
    }
  }
  return out;
}

int runSe3() {
  std::string line;
  while (std::getline(std::cin, line)) {
    const Json record = Json::parse(line, nullptr, false);
    std::vector<double> a;
    std::vector<double> b;
    if (record.is_discarded() || !readArray(record, "a", kWxyzXyzSize, a) ||
        !readArray(record, "b", kWxyzXyzSize, b)) {
      return EXIT_FAILURE;
    }
    const FramePose pa = fromWxyzXyz(a);
    const FramePose pb = fromWxyzXyz(b);
    const FramePose inv_mul = openarm_ik::se3InverseMultiply(pa, pb);
    const Eigen::Quaterniond& q = inv_mul.orientation;
    std::cout << Json{{"log_a", rowMajor(openarm_ik::se3Log(pa))},
                      {"rminus", rowMajor(openarm_ik::se3RightMinus(pa, pb))},
                      {"inv_mul",
                       {q.w(), q.x(), q.y(), q.z(), inv_mul.position.x(), inv_mul.position.y(),
                        inv_mul.position.z()}},
                      {"jlog_a", rowMajor(openarm_ik::se3Jlog(pa))}}
                     .dump()
              << '\n';
  }
  return EXIT_SUCCESS;
}

int runBoxQp() {
  constexpr std::size_t kN = openarm_ik::kNumArmJoints;
  std::string line;
  while (std::getline(std::cin, line)) {
    const Json record = Json::parse(line, nullptr, false);
    std::vector<double> h;
    std::vector<double> c;
    std::vector<double> lo;
    std::vector<double> hi;
    if (record.is_discarded() || !readArray(record, "H", kN * kN, h) ||
        !readArray(record, "c", kN, c) || !readArray(record, "lo", kN, lo) ||
        !readArray(record, "hi", kN, hi)) {
      return EXIT_FAILURE;
    }
    const openarm_ik::JointMatrix hessian =
        Eigen::Map<const Eigen::Matrix<double, kN, kN, Eigen::RowMajor>>(h.data());
    const openarm_ik::BoxQpResult result =
        openarm_ik::solveBoxQp(hessian, Eigen::Map<const openarm_ik::JointVector>(c.data()),
                               Eigen::Map<const openarm_ik::JointVector>(lo.data()),
                               Eigen::Map<const openarm_ik::JointVector>(hi.data()));
    std::cout << Json{{"x", rowMajor(result.solution)},
                      {"status", openarm_ik::boxQpStatusName(result.status)},
                      {"iterations", result.iterations}}
                     .dump()
              << '\n';
  }
  return EXIT_SUCCESS;
}

int runReplay(const char* xml_path) {
  openarm_ik::ArmKinematics::Config config;
  config.xml_path = xml_path;
  const auto kinematics = openarm_ik::ArmKinematics::create(config);
  if (kinematics == nullptr) {
    return EXIT_FAILURE;  // create() logged why
  }
  constexpr std::array<ArmSide, 2> kBimanual = {ArmSide::kRight, ArmSide::kLeft};
  const auto solver = openarm_ik::IkSolver::create(*kinematics, {}, kBimanual);
  if (solver == nullptr) {
    return EXIT_FAILURE;
  }
  std::string line;
  while (std::getline(std::cin, line)) {
    const Json record = Json::parse(line, nullptr, false);
    if (record.is_discarded()) {
      logf(LogLevel::kError, "replay: invalid JSON line");
      return EXIT_FAILURE;
    }
    for (const ArmSide side : kBimanual) {
      const char* key = openarm_ik::sideName(side);
      if (!record.contains(key)) {
        continue;  // no new target for this arm on this tick
      }
      std::vector<double> pose;
      if (!readArray(record, key, kPoseSize, pose)) {
        return EXIT_FAILURE;
      }
      solver->setTarget(side, fromNodePose(pose));
    }
    const openarm_ik::IkStatus status =
        solver->ready() ? solver->solve() : openarm_ik::IkStatus::kNotReady;
    std::cout << Json{{"status", openarm_ik::ikStatusName(status)},
                      {"right", rowMajor(solver->jointPositions(ArmSide::kRight))},
                      {"left", rowMajor(solver->jointPositions(ArmSide::kLeft))}}
                     .dump()
              << '\n';
  }
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  quest_vr::setMinLogLevel(LogLevel::kWarn);
  const std::span<char*> args(argv, static_cast<std::size_t>(argc));
  if (args.size() < 2) {
    std::cerr << "usage: ik_check se3 | boxqp | replay <scene.xml>\n";
    return EXIT_FAILURE;
  }
  const std::string_view mode = args[1];
  if (mode == "se3") {
    return runSe3();
  }
  if (mode == "boxqp") {
    return runBoxQp();
  }
  if (mode == "replay" && args.size() >= 3) {
    return runReplay(args[2]);
  }
  logf(LogLevel::kError, "unknown mode '{}' (or missing scene.xml for replay)", mode);
  return EXIT_FAILURE;
}
