// Timing of IkSolver::solve() for the bimanual teleop case, single thread vs
// one thread per arm. Targets come from FK of a smooth joint-space motion
// (like tests/check_ik.py), so every solve does real work.
//
// Usage: ik_bench <scene.xml> [num_solves] [daqp|activeset]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "openarm_ik/ik_solver.hpp"
#include "quest_vr/logging.hpp"

namespace {

using openarm_ik::ArmSide;
using Clock = std::chrono::steady_clock;

constexpr int kDefaultSolves = 20000;
constexpr int kWarmupSolves = 200;
constexpr double kAmplitudeOfHalfRange = 0.35;  // fraction of each joint's half range
constexpr double kTicksPerCycle = 2000.0;       // motion period in solves (4 s at 500 Hz)
constexpr double kMicrosPerSecond = 1e6;
constexpr double kPercentile50 = 0.50;
constexpr double kPercentile99 = 0.99;
constexpr double kPercentile999 = 0.999;

struct Summary {
  double mean_us, p50_us, p99_us, p999_us, max_us;
};

Summary summarize(std::vector<double> samples) {
  std::sort(samples.begin(), samples.end());
  const auto at = [&](double p) {
    return samples[static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1))];
  };
  double sum = 0.0;
  for (const double s : samples) {
    sum += s;
  }
  return {sum / static_cast<double>(samples.size()), at(kPercentile50), at(kPercentile99),
          at(kPercentile999), samples.back()};
}

// Joint-space trajectory → TCP targets, computed once with a scratch model.
std::vector<std::array<openarm_ik::FramePose, 2>> makeTargets(const std::string& xml, int count) {
  openarm_ik::ArmKinematics::Config config;
  config.xml_path = xml;
  auto kin = openarm_ik::ArmKinematics::create(config);
  std::vector<std::array<openarm_ik::FramePose, 2>> targets;
  if (kin == nullptr) {
    return targets;
  }
  targets.reserve(static_cast<std::size_t>(count));
  for (int t = 0; t < count; ++t) {
    std::array<openarm_ik::FramePose, 2> pair{};
    for (const ArmSide side : openarm_ik::kArmSides) {
      const auto& idx = kin->indices(side);
      openarm_ik::JointVector q;
      for (Eigen::Index j = 0; j < q.size(); ++j) {
        const double mid = 0.5 * (idx.lower(j) + idx.upper(j));
        const double half = 0.5 * (idx.upper(j) - idx.lower(j));
        const double phase = static_cast<double>(j) + (side == ArmSide::kLeft ? 1.0 : 0.0);
        q(j) = mid + kAmplitudeOfHalfRange * half *
                         std::sin(2.0 * std::numbers::pi * t / kTicksPerCycle * (1.0 + 0.1 * j) + phase);
      }
      kin->setJointPositions(side, q);
      pair[openarm_ik::sideIndex(side)] = kin->tcpPose(side);
    }
    targets.push_back(pair);
  }
  return targets;
}

bool bench(const std::string& xml, openarm_ik::IkThreading threading,
           openarm_ik::QpBackend backend, const std::vector<std::array<openarm_ik::FramePose, 2>>& targets) {
  openarm_ik::ArmKinematics::Config config;
  config.xml_path = xml;
  openarm_ik::IkParams params;
  params.threading = threading;
  params.qp_backend = backend;
  constexpr std::array<ArmSide, 2> kBimanual = {ArmSide::kRight, ArmSide::kLeft};
  auto solver = openarm_ik::IkSolver::create(config, params, kBimanual);
  if (solver == nullptr) {
    return false;
  }
  std::vector<double> samples;
  samples.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    for (const ArmSide side : kBimanual) {
      solver->setTarget(side, targets[i][openarm_ik::sideIndex(side)]);
    }
    const auto start = Clock::now();
    const auto status = solver->solve();
    const double us = std::chrono::duration<double>(Clock::now() - start).count() * kMicrosPerSecond;
    if (status != openarm_ik::IkStatus::kSolved) {
      std::cerr << "solve failed at " << i << '\n';
      return false;
    }
    if (i >= static_cast<std::size_t>(kWarmupSolves)) {
      samples.push_back(us);
    }
  }
  const Summary s = summarize(samples);
  std::cout << std::format("{:<16} {:<10} mean {:6.1f}  p50 {:6.1f}  p99 {:6.1f}  p99.9 {:6.1f}  max {:7.1f} us\n",
                           threading == openarm_ik::IkThreading::kThreadPerArm ? "thread per arm" : "single thread",
                           openarm_ik::qpBackendName(backend), s.mean_us, s.p50_us, s.p99_us, s.p999_us, s.max_us);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  quest_vr::setMinLogLevel(quest_vr::LogLevel::kWarn);
  const std::span<char*> args(argv, static_cast<std::size_t>(argc));
  if (args.size() < 2) {
    std::cerr << "usage: ik_bench <scene.xml> [num_solves] [daqp|activeset]\n";
    return EXIT_FAILURE;
  }
  const std::string xml = args[1];
  const int solves = args.size() > 2 ? std::stoi(args[2]) : kDefaultSolves;
  const auto targets = makeTargets(xml, solves + kWarmupSolves);
  if (targets.empty()) {
    return EXIT_FAILURE;
  }
  std::cout << std::format("{} bimanual solves (10 iterations each), after {} warm-up\n", solves, kWarmupSolves);
  for (const auto backend : {openarm_ik::QpBackend::kDaqp, openarm_ik::QpBackend::kActiveSet}) {
    if (args.size() > 3 && std::string(args[3]) != openarm_ik::qpBackendName(backend)) {
      continue;
    }
    for (const auto threading : {openarm_ik::IkThreading::kSingleThread, openarm_ik::IkThreading::kThreadPerArm}) {
      if (!bench(xml, threading, backend, targets)) {
        return EXIT_FAILURE;
      }
    }
  }
  return EXIT_SUCCESS;
}
