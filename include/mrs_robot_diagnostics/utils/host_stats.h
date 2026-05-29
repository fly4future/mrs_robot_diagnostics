#pragma once

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <mrs_msgs/msg/cpu_load.hpp>

namespace mrs_robot_diagnostics::utils
{

/**
 * @brief /proc-backed host-machine stats reader.
 *
 * Collects CPU load / frequency / temperature, RAM usage, disk space, and
 * per-PID CPU loads. Maintains incremental state for CPU-tick diffs between
 * successive update() calls.
 *
 * * Per-PID CPU loads are approximated by attributing each ROS2-linked PID
  * (detected via `librclcpp.so` in /proc/<pid>/maps) a top-style CPU % based on
  * utime+stime deltas between successive update() calls.
 */

class HostStats {
public:
  /**
   * @brief Snapshot of all currently-known host stats. Values default to
   * "unknown" sentinels (-1) until update() succeeds.
   */
  struct Snapshot
  {
    float                               cpu_load        = -1.0f; // [%]
    float                               cpu_ghz         = -1.0f; // [GHz]
    float                               cpu_temperature = -1.0f; // [°C]
    float                               free_ram        = -1.0f; // [GiB]
    float                               total_ram       = -1.0f; // [GiB]
    int32_t                             free_hdd        = -1;    // [GiB]
    std::vector<mrs_msgs::msg::CpuLoad> node_cpu_loads;

    // WiFi (empty interface == unavailable).
    std::string wifi_interface;
    float       wifi_signal_dbm   = std::numeric_limits<float>::quiet_NaN(); // [dBm]
    int32_t     wifi_link_quality = -1;                                      // 0–70 typical
  };

  /**
   * @brief Restrict WiFi reads to a specific interface (e.g. "wlp2s0").
   * Empty value (the default) means: auto-pick the first interface listed in /proc/net/wireless.
   */
  void setWifiInterface(std::string iface);

  /** @brief Refresh all stats from /proc and /sys. Safe to call from any single thread. */
  void update();

  /** @brief Read the latest snapshot. */
  Snapshot snapshot() const;

private:
  void readCpuLoad();
  void readCpuTemperature();
  void readCpuFreq();
  void readMemLoad();
  void readDiskSpace();
  void readNodeCpuLoads();
  void readWifi();

  mutable std::mutex mutex_;
  Snapshot           snap_;
  std::string        wifi_interface_filter_; // empty = first interface in /proc/net/wireless

  // CPU diff state carried across readCpuLoad() invocations.
  long cpu_last_idle_       = 0;
  long cpu_last_total_      = 0;
  long cpu_total_diff_      = 0;
  long cpu_last_total_diff_ = 0;
  int  cpu_cores_           = 1;

  // Per-PID CPU diff state carried across readNodeCpuLoads() invocations.
  // proc_last_ticks_ stores last-seen utime+stime for each tracked PID.
  // pid_is_ros_ caches the "is this PID a ROS2 process?" decision so we don't
  // re-scan /proc/<pid>/maps on every tick. Both are pruned for dead PIDs.
  std::unordered_map<int, long> proc_last_ticks_;
  std::unordered_map<int, bool> pid_is_ros_;
};

} // namespace mrs_robot_diagnostics::utils
