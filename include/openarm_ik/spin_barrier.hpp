// Reusable barrier for a fixed number of threads that busy-waits instead of
// sleeping. The IK workers meet 2× per iteration (20× per solve, ~3 µs apart):
// a futex-based std::barrier would put a thread to sleep and pay ~10–50 µs to
// wake it each time, more than the work in between.
#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

namespace openarm_ik {

class SpinBarrier {
 public:
  explicit SpinBarrier(int parties) : parties_(parties) {}

  void arriveAndWait() {
    const std::uint64_t generation = generation_.load(std::memory_order_acquire);
    if (arrived_.fetch_add(1, std::memory_order_acq_rel) == parties_ - 1) {
      arrived_.store(0, std::memory_order_relaxed);
      generation_.fetch_add(1, std::memory_order_acq_rel);  // releases everyone
      return;
    }
    int spins = 0;
    while (generation_.load(std::memory_order_acquire) == generation) {
      if (++spins > kSpinsBeforeYield) {
        std::this_thread::yield();  // the other thread was descheduled; let it run
      }
    }
  }

 private:
  static constexpr int kSpinsBeforeYield = 4096;
  const int parties_;
  std::atomic<int> arrived_{0};
  std::atomic<std::uint64_t> generation_{0};
};

}  // namespace openarm_ik
