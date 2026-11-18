// Test harness for step 1 (ArmKinematics): FK + body Jacobian for given joints.
//
// Usage: ik_kinematics_dump <scene.xml> [input.jsonl]
// Input  lines: {"right": [7 joint angles], "left": [7 joint angles]}
// Output lines: {"right": {"pos": [3], "quat_wxyz": [4], "jac": [6×7 row-major]}, "left": {...}}
// The first output line is {"indices": ...} describing the resolved model layout.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "openarm_ik/arm_kinematics.hpp"
#include "quest_vr/logging.hpp"

namespace {

using Json = nlohmann::json;
using openarm_ik::ArmSide;
using quest_vr::LogLevel;
using quest_vr::logf;

Json indicesToJson(const openarm_ik::ArmIndices& arm) {
  return {{"qpos", arm.qpos},
          {"dof", arm.dof},
          {"lower", std::vector<double>(arm.lower.data(), arm.lower.data() + arm.lower.size())},
          {"upper", std::vector<double>(arm.upper.data(), arm.upper.data() + arm.upper.size())},
          {"tcp_body", arm.tcp_body}};
}

Json armToJson(const openarm_ik::ArmKinematics& kinematics, ArmSide side) {
  const openarm_ik::FramePose pose = kinematics.tcpPose(side);
  const openarm_ik::ArmJacobian jacobian = kinematics.tcpJacobianLocal(side);
  std::vector<double> jac_row_major;
  jac_row_major.reserve(static_cast<std::size_t>(jacobian.size()));
  for (Eigen::Index row = 0; row < jacobian.rows(); ++row) {
    for (Eigen::Index col = 0; col < jacobian.cols(); ++col) {
      jac_row_major.push_back(jacobian(row, col));
    }
  }
  const Eigen::Quaterniond& q = pose.orientation;
  return {{"pos", {pose.position.x(), pose.position.y(), pose.position.z()}},
          {"quat_wxyz", {q.w(), q.x(), q.y(), q.z()}},
          {"jac", jac_row_major}};
}

bool readJoints(const Json& record, const char* key, openarm_ik::JointVector& out) {
  if (!record.contains(key) || !record[key].is_array() ||
      record[key].size() != openarm_ik::kNumArmJoints) {
    logf(LogLevel::kError, "input record needs '{}': array of {} numbers", key,
         openarm_ik::kNumArmJoints);
    return false;
  }
  for (std::size_t i = 0; i < openarm_ik::kNumArmJoints; ++i) {
    out(static_cast<Eigen::Index>(i)) = record[key][i].get<double>();
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  quest_vr::setMinLogLevel(LogLevel::kWarn);
  if (argc < 2) {
    std::cerr << "usage: ik_kinematics_dump <scene.xml> [input.jsonl]\n";
    return EXIT_FAILURE;
  }
  openarm_ik::ArmKinematics::Config config;
  config.xml_path = argv[1];
  const auto kinematics = openarm_ik::ArmKinematics::create(config);
  if (kinematics == nullptr) {
    return EXIT_FAILURE;  // create() logged why
  }

  std::ifstream file;
  if (argc > 2) {
    file.open(argv[2]);
    if (!file) {
      logf(LogLevel::kError, "cannot open '{}'", argv[2]);
      return EXIT_FAILURE;
    }
  }
  std::istream& input = argc > 2 ? static_cast<std::istream&>(file) : std::cin;

  std::cout << Json{{"indices",
                     {{"right", indicesToJson(kinematics->indices(ArmSide::kRight))},
                      {"left", indicesToJson(kinematics->indices(ArmSide::kLeft))}}}}
                   .dump()
            << '\n';

  std::string line;
  while (std::getline(input, line)) {
    const Json record = Json::parse(line, nullptr, false);
    openarm_ik::JointVector q_right;
    openarm_ik::JointVector q_left;
    if (record.is_discarded() || !readJoints(record, "right", q_right) ||
        !readJoints(record, "left", q_left)) {
      return EXIT_FAILURE;
    }
    if (!kinematics->setJointPositions(ArmSide::kRight, q_right) ||
        !kinematics->setJointPositions(ArmSide::kLeft, q_left)) {
      return EXIT_FAILURE;
    }
    std::cout << Json{{"right", armToJson(*kinematics, ArmSide::kRight)},
                      {"left", armToJson(*kinematics, ArmSide::kLeft)}}
                     .dump()
              << '\n';
  }
  return EXIT_SUCCESS;
}
