#include "quest_vr/packet.hpp"

#include <array>
#include <cmath>
#include <format>
#include <nlohmann/json.hpp>

namespace quest_vr {
namespace {

using Json = nlohmann::json;

void addIssue(ParseIssues* issues, std::string message) {
  if (issues != nullptr) {
    issues->push_back(std::move(message));
  }
}

std::string_view trim(std::string_view text) {
  constexpr std::string_view kWhitespace = " \t\r\n\f\v";
  const auto first = text.find_first_not_of(kWhitespace);
  if (first == std::string_view::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(kWhitespace);
  return text.substr(first, last - first + 1);
}

// Absent key → nullopt silently. Present but not a finite number → nullopt + issue.
std::optional<double> readNumber(const Json& object, const std::string& key,
                                 ParseIssues* issues) {
  const auto it = object.find(key);
  if (it == object.end()) {
    return std::nullopt;
  }
  if (!it->is_number()) {
    addIssue(issues, std::format("'{}' is not a number ({})", key, it->type_name()));
    return std::nullopt;
  }
  const double value = it->get<double>();
  if (!std::isfinite(value)) {
    addIssue(issues, std::format("'{}' is not finite", key));
    return std::nullopt;
  }
  return value;
}

// Buttons may arrive as JSON bool or as 0/1 (python: bool(msg["a"])).
std::optional<bool> readBool(const Json& object, const std::string& key, ParseIssues* issues) {
  const auto it = object.find(key);
  if (it == object.end()) {
    return std::nullopt;
  }
  if (it->is_boolean()) {
    return it->get<bool>();
  }
  if (it->is_number()) {
    return it->get<double>() != 0.0;
  }
  addIssue(issues, std::format("'{}' is not a bool/number ({})", key, it->type_name()));
  return std::nullopt;
}

// Unknown validity codes are treated as INVALID: the safe choice, since an
// unknown state must never drive the arm.
std::optional<Validity> readValidity(const Json& object, const std::string& key,
                                     ParseIssues* issues) {
  const auto raw = readNumber(object, key, issues);
  if (!raw.has_value()) {
    return std::nullopt;
  }
  const double code = *raw;
  if (code == static_cast<double>(Validity::kOk)) {
    return Validity::kOk;
  }
  if (code == static_cast<double>(Validity::kStale)) {
    return Validity::kStale;
  }
  if (code == static_cast<double>(Validity::kInvalid)) {
    return Validity::kInvalid;
  }
  addIssue(issues, std::format("'{}' has unknown validity code {}, treating as INVALID", key, code));
  return Validity::kInvalid;
}

// A pose is accepted only if all 7 fields are present and finite: a partial
// pose would otherwise silently default some components to 0.
std::optional<UnityPose> readPose(const Json& object, const std::string& key,
                                  ParseIssues* issues) {
  const auto it = object.find(key);
  if (it == object.end()) {
    return std::nullopt;
  }
  if (!it->is_object()) {
    addIssue(issues, std::format("'{}' is not an object ({})", key, it->type_name()));
    return std::nullopt;
  }
  struct Field {
    const char* name;
    double UnityPose::*member;
  };
  constexpr std::array<Field, kFlatPoseSize> kFields = {{
      {"x", &UnityPose::x},
      {"y", &UnityPose::y},
      {"z", &UnityPose::z},
      {"qx", &UnityPose::qx},
      {"qy", &UnityPose::qy},
      {"qz", &UnityPose::qz},
      {"qw", &UnityPose::qw},
  }};
  UnityPose pose;
  for (const Field& field : kFields) {
    const auto value = readNumber(*it, field.name, issues);
    if (!value.has_value()) {
      addIssue(issues, std::format("'{}' dropped: field '{}' missing or invalid", key, field.name));
      return std::nullopt;
    }
    pose.*field.member = *value;
  }
  return pose;
}

}  // namespace

std::optional<QuestPacket> parseQuestPacket(std::string_view text, ParseIssues* issues) {
  const std::string_view body = trim(text);
  if (body.empty()) {
    addIssue(issues, "empty datagram");
    return std::nullopt;
  }
  const Json root = Json::parse(body, /*cb=*/nullptr, /*allow_exceptions=*/false);
  if (root.is_discarded()) {
    addIssue(issues, "datagram is not valid JSON");
    return std::nullopt;
  }
  if (!root.is_object()) {
    addIssue(issues, std::format("datagram JSON is not an object ({})", root.type_name()));
    return std::nullopt;
  }

  QuestPacket packet;
  packet.t = readNumber(root, "t", issues);
  packet.left_controller = readPose(root, "lc", issues);
  packet.right_controller = readPose(root, "rc", issues);
  packet.reference = readPose(root, "rf", issues);
  packet.left_trigger = readNumber(root, "lt", issues);
  packet.right_trigger = readNumber(root, "rt", issues);
  packet.left_grip = readNumber(root, "lg", issues);
  packet.right_grip = readNumber(root, "rg", issues);
  packet.left_stick_x = readNumber(root, "lsx", issues);
  packet.left_stick_y = readNumber(root, "lsy", issues);
  packet.right_stick_x = readNumber(root, "rsx", issues);
  packet.right_stick_y = readNumber(root, "rsy", issues);
  packet.button_a = readBool(root, "a", issues);
  packet.button_b = readBool(root, "b", issues);
  packet.button_x = readBool(root, "x", issues);
  packet.button_y = readBool(root, "y", issues);
  packet.validity = readValidity(root, "v", issues);
  packet.left_validity = readValidity(root, "vl", issues);
  packet.right_validity = readValidity(root, "vr", issues);
  return packet;
}

}  // namespace quest_vr
