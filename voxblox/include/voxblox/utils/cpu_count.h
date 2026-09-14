#ifndef VOXBLOX_UTILS_CPU_COUNT_H_
#define VOXBLOX_UTILS_CPU_COUNT_H_

#include <algorithm>
#include <cstdlib>
#include <thread>

#ifdef __linux__
#include <sched.h>
#endif

namespace voxblox {

/// Returns OMP_NUM_THREADS as a positive count, or zero when unset or unparsable.
inline size_t threadCapFromEnvironment() {
  const char* const value = std::getenv("OMP_NUM_THREADS");
  if (value == nullptr) {
    return 0u;
  }

  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);  // NOLINT(runtime/int)
  if (end == value || parsed <= 0) {
    return 0u;
  }

  return static_cast<size_t>(parsed);
}

/**
 * Returns the number of CPUs this process may actually run on, which is at least
 * one. std::thread::hardware_concurrency() reports the machine's online CPUs and
 * ignores both the affinity mask and the cpuset cgroup, so it overcounts badly
 * under taskset, Docker or a batch-system allocation: a 16-CPU Slurm job on a
 * 128-core node sees 128. The affinity mask reports the allocation instead.
 * OMP_NUM_THREADS, when set, caps the result.
 */
inline size_t availableCpuCount() {
  size_t available = 0u;

#ifdef __linux__
  cpu_set_t mask;
  CPU_ZERO(&mask);
  if (sched_getaffinity(0, sizeof(mask), &mask) == 0) {
    available = static_cast<size_t>(CPU_COUNT(&mask));
  }
#endif

  if (available == 0u) {
    available = static_cast<size_t>(std::thread::hardware_concurrency());
  }

  const size_t cap = threadCapFromEnvironment();
  if (cap != 0u) {
    available = std::min(available, cap);
  }

  return std::max<size_t>(1u, available);
}

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_CPU_COUNT_H_
