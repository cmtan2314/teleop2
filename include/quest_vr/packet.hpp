// Parsed Meta Quest UDP packet (one JSON object per datagram).
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "quest_vr/types.hpp"

namespace quest_vr {

// Every field is optional: the sender may omit any of them, and the python
// node only acts on the ones present ("if 'rt' in msg: ...").
struct QuestPacket {
  std::optional<double> t;  // headset timestamp (s)

  std::optional<UnityPose> left_controller;   // "lc"
  std::optional<UnityPose> right_controller;  // "rc"
  std::optional<UnityPose> reference;         // "rf"

  std::optional<double> left_trigger;   // "lt" 0..1
  std::optional<double> right_trigger;  // "rt" 0..1
  std::optional<double> left_grip;      // "lg" 0..1
  std::optional<double> right_grip;     // "rg" 0..1

  std::optional<double> left_stick_x;   // "lsx" -1..1
  std::optional<double> left_stick_y;   // "lsy" -1..1
  std::optional<double> right_stick_x;  // "rsx" -1..1
  std::optional<double> right_stick_y;  // "rsy" -1..1

  std::optional<bool> button_a;
  std::optional<bool> button_b;
  std::optional<bool> button_x;
  std::optional<bool> button_y;

  std::optional<Validity> validity;        // "v"
  std::optional<Validity> left_validity;   // "vl"
  std::optional<Validity> right_validity;  // "vr"
};

// Human-readable problems found while parsing (bad field types, NaN, ...).
using ParseIssues = std::vector<std::string>;

// Parse one datagram. Returns nullopt if the text is empty, not JSON, or not a
// JSON object. Individually malformed fields are dropped (treated as absent)
// and described in `issues` when it is non-null.
std::optional<QuestPacket> parseQuestPacket(std::string_view text,
                                            ParseIssues* issues = nullptr);

}  // namespace quest_vr
