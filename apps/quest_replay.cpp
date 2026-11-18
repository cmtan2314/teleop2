// Offline replay for parity testing against the python implementation.
// Input  (stdin or file): one JSON object per line: {"t": <now_s>, "msg": {<quest packet>}}
// Output (stdout):        one JSON object per line with the resulting TeleopFrame.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "quest_vr/logging.hpp"
#include "quest_vr/teleop_input.hpp"

namespace {

using Json = nlohmann::json;
using quest_vr::LogLevel;
using quest_vr::logf;

Json poseToJson(const std::optional<quest_vr::Pose>& pose) {
  if (!pose.has_value()) {
    return nullptr;
  }
  const auto flat = quest_vr::toFlatPose(*pose);
  return Json(std::vector<double>(flat.begin(), flat.end()));
}

Json armToJson(const quest_vr::ArmFrame& arm) {
  return {{"pose", poseToJson(arm.pose)},
          {"engaged", arm.engaged},
          {"timer_running", arm.grip_timer_running}};
}

}  // namespace

int main(int argc, char** argv) {
  quest_vr::setMinLogLevel(LogLevel::kWarn);  // keep stderr quiet; stdout carries the data
  std::ifstream file;
  if (argc > 1) {
    file.open(argv[1]);
    if (!file) {
      logf(LogLevel::kError, "cannot open '{}'", argv[1]);
      return EXIT_FAILURE;
    }
  }
  std::istream& input = argc > 1 ? static_cast<std::istream&>(file) : std::cin;

  quest_vr::QuestTeleopInput teleop(quest_vr::QuestPoseProcessor(std::nullopt));
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const Json record = Json::parse(line, nullptr, false);
    if (record.is_discarded() || !record.contains("t") || !record.contains("msg") ||
        !record["t"].is_number()) {
      logf(LogLevel::kError, "line {}: expected {{\"t\": number, \"msg\": object}}", line_number);
      return EXIT_FAILURE;
    }
    const auto packet = quest_vr::parseQuestPacket(record["msg"].dump());
    if (!packet.has_value()) {
      logf(LogLevel::kError, "line {}: packet rejected by parser", line_number);
      return EXIT_FAILURE;
    }
    const quest_vr::TeleopFrame frame = teleop.step(*packet, record["t"].get<double>());
    const Json out = {{"right", armToJson(frame.right)}, {"left", armToJson(frame.left)}};
    std::cout << out.dump() << '\n';
  }
  return EXIT_SUCCESS;
}
