// Reads config/grasp_presets.yaml (same file the python node takes with
// --grasp-presets). Unlike python, every mistake is reported at start-up:
// python would raise only on an unknown cycle.order name and silently never
// cycle on a misspelled button.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>

#include "quest_vr/logging.hpp"
#include "teleop_bridge/bridge_logic.hpp"

namespace teleop_bridge {
namespace {

using quest_vr::LogLevel;
using quest_vr::logf;

constexpr std::size_t kMinCycleLength = 2;

std::optional<VrButton> parseButton(const std::string& name) {
  for (const VrButton b : {VrButton::kA, VrButton::kB, VrButton::kX, VrButton::kY}) {
    if (name == buttonName(b)) {
      return b;
    }
  }
  return std::nullopt;
}

std::optional<ArmSide> parseSide(const std::string& name) {
  for (const ArmSide side : openarm_ik::kArmSides) {
    if (name == openarm_ik::sideName(side)) {
      return side;
    }
  }
  return std::nullopt;
}

}  // namespace

GraspConfig defaultGraspConfig() {
  GraspConfig config;
  GraspPreset box;
  box.name = std::string(kDefaultPresetName);
  box.closed = Eigen::Map<const HandVector>(kDefaultHandClosedRad.data());
  config.presets.push_back(box);
  return config;
}

std::optional<GraspConfig> loadGraspConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& error) {
    logf(LogLevel::kError, "grasp presets: cannot read '{}': {}", path, error.what());
    return std::nullopt;
  }
  try {
    GraspConfig config;
    const YAML::Node presets = root["presets"];
    if (!presets || !presets.IsMap() || presets.size() == 0) {
      logf(LogLevel::kError, "grasp presets '{}': needs a non-empty 'presets:' map", path);
      return std::nullopt;
    }
    for (const auto& entry : presets) {  // yaml-cpp keeps the file order
      GraspPreset preset;
      preset.name = entry.first.as<std::string>();
      const auto values = entry.second.as<std::vector<double>>();
      if (values.size() != kNumHandJoints) {
        logf(LogLevel::kError, "grasp presets '{}': preset '{}' has {} values, expected {}", path,
             preset.name, values.size(), kNumHandJoints);
        return std::nullopt;
      }
      preset.closed = Eigen::Map<const HandVector>(values.data());
      if (!preset.closed.allFinite()) {
        logf(LogLevel::kError, "grasp presets '{}': preset '{}' is not finite", path, preset.name);
        return std::nullopt;
      }
      config.presets.push_back(preset);
    }

    const YAML::Node cycle = root["cycle"];
    if (!cycle || cycle.IsNull()) {
      logf(LogLevel::kInfo, "grasp presets '{}': {} preset(s), no cycle button", path,
           config.presets.size());
      return config;
    }
    PresetCycle parsed;
    const auto button_name = cycle["button"].as<std::string>();
    const auto button = parseButton(button_name);
    if (!button.has_value()) {
      logf(LogLevel::kError, "grasp presets '{}': cycle.button '{}' is not a|b|x|y", path,
           button_name);
      return std::nullopt;
    }
    parsed.button = *button;
    const auto hands = cycle["hands"] ? cycle["hands"].as<std::vector<std::string>>()
                                      : std::vector<std::string>{"left", "right"};
    for (const std::string& name : hands) {
      const auto side = parseSide(name);
      if (!side.has_value()) {
        logf(LogLevel::kError, "grasp presets '{}': cycle.hands entry '{}' is not left|right",
             path, name);
        return std::nullopt;
      }
      parsed.hands.push_back(*side);
    }
    parsed.order = cycle["order"].as<std::vector<std::string>>();
    if (parsed.order.size() < kMinCycleLength) {
      logf(LogLevel::kError, "grasp presets '{}': cycle.order needs at least {} names", path,
           kMinCycleLength);
      return std::nullopt;
    }
    for (const std::string& name : parsed.order) {
      const bool known = std::any_of(config.presets.begin(), config.presets.end(),
                                     [&](const GraspPreset& p) { return p.name == name; });
      if (!known) {
        logf(LogLevel::kError, "grasp presets '{}': cycle.order: unknown preset '{}'", path,
             name);
        return std::nullopt;
      }
    }
    config.cycle = parsed;
    logf(LogLevel::kInfo, "grasp presets '{}': {} preset(s), button {} cycles {} name(s)", path,
         config.presets.size(), buttonName(parsed.button), parsed.order.size());
    return config;
  } catch (const std::exception& error) {
    logf(LogLevel::kError, "grasp presets '{}': malformed: {}", path, error.what());
    return std::nullopt;
  }
}

}  // namespace teleop_bridge
