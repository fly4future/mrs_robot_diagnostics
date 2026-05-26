#include <mrs_robot_diagnostics/utils/host_stats.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mrs_robot_diagnostics::utils
{

namespace
{

std::vector<std::string> splitByChar(const std::string &line, char delim) {
  std::vector<std::string> out;
  std::stringstream        ss(line);
  std::string              item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

// Returns true if /proc/<pid>/maps contains a librclcpp.so mapping —
// a reliable signal that the process linked the ROS2 C++ client library.
// Returns false on EACCES (other-user process) or any read error.
bool isRosProcess(int pid) {
  std::ifstream maps("/proc/" + std::to_string(pid) + "/maps");
  if (!maps.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("librclcpp.so") != std::string::npos) {
      return true;
    }
  }
  return false;
}

// Reads utime + stime (jiffies) from /proc/<pid>/stat. Returns -1 on failure.
// The comm field (parens-wrapped) can contain spaces and parens, so we
// anchor parsing on the final ')' rather than tokenizing the whole line.
long readProcTicks(int pid) {
  std::ifstream stat("/proc/" + std::to_string(pid) + "/stat");
  std::string   line;
  if (!std::getline(stat, line)) {
    return -1;
  }
  const auto rparen = line.rfind(')');
  if (rparen == std::string::npos || rparen + 2 > line.size()) {
    return -1;
  }
  // Tokens after ')' begin at field 3 (state); utime is field 14 → index 11, stime → index 12.
  const auto tokens = splitByChar(line.substr(rparen + 2), ' ');
  if (tokens.size() < 13) {
    return -1;
  }
  try {
    return std::stol(tokens[11]) + std::stol(tokens[12]);
  }
  catch (const std::exception &) {
    return -1;
  }
}

// Best-effort process label. Prefers __node:=<name> from /proc/<pid>/cmdline
// (so we get the actual ROS node name instead of "component_container_mt"),
// falls back to /proc/<pid>/comm. Returns "pid<N>" if both fail.
std::string readProcName(int pid) {
  std::ifstream cmdline("/proc/" + std::to_string(pid) + "/cmdline");
  if (cmdline.is_open()) {
    std::string raw((std::istreambuf_iterator<char>(cmdline)), std::istreambuf_iterator<char>());
    // cmdline tokens are NUL-separated.
    const std::string marker = "__node:=";
    const auto        pos    = raw.find(marker);
    if (pos != std::string::npos) {
      const auto end = raw.find('\0', pos);
      return raw.substr(pos + marker.size(), end - pos - marker.size());
    }
  }
  std::ifstream comm("/proc/" + std::to_string(pid) + "/comm");
  std::string   name;
  if (std::getline(comm, name) && !name.empty()) {
    return name;
  }
  return "pid" + std::to_string(pid);
}

} // namespace

void HostStats::setWifiInterface(std::string iface) {
  std::scoped_lock lock(mutex_);
  wifi_interface_filter_ = std::move(iface);
}

void HostStats::update() {
  std::scoped_lock lock(mutex_);
  readCpuFreq();
  readCpuLoad();
  readCpuTemperature();
  readMemLoad();
  readDiskSpace();
  readNodeCpuLoads();
  readWifi();
}

HostStats::Snapshot HostStats::snapshot() const {
  std::scoped_lock lock(mutex_);
  return snap_;
}

void HostStats::readCpuLoad() {
  std::ifstream file("/proc/stat");
  std::string   line;
  if (!std::getline(file, line)) {
    return;
  }

  const auto results = splitByChar(line, ' ');

  long idle     = 0;
  long non_idle = 0;
  long total    = 0;

  try {
    // /proc/stat format: "cpu  user nice system idle iowait irq softirq steal ..."
    // After splitByChar, results[0]="cpu", results[1]="" (double space), values shift by one.
    idle     = std::stol(results.at(5)) + std::stol(results.at(6));
    non_idle = std::stol(results.at(2)) + std::stol(results.at(3)) + std::stol(results.at(4)) + std::stol(results.at(7)) + std::stol(results.at(8)) +
               std::stol(results.at(9));
    total = idle + non_idle;
  }
  catch (const std::exception &) {
    return;
  }

  cpu_last_total_diff_ = cpu_total_diff_;
  cpu_total_diff_      = total - cpu_last_total_;
  const long idle_diff = idle - cpu_last_idle_;

  if (cpu_total_diff_ > 0) {
    snap_.cpu_load = 100.0f * static_cast<float>(cpu_total_diff_ - idle_diff) / static_cast<float>(cpu_total_diff_);
  }

  cpu_last_total_ = total;
  cpu_last_idle_  = idle;
}

void HostStats::readCpuTemperature() {
  long max_temp = 0;
  for (int i = 0; i < 11; ++i) {
    const std::string path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
    if (!std::filesystem::exists(path)) {
      break;
    }
    std::ifstream file(path);
    std::string   line;
    if (!std::getline(file, line)) {
      continue;
    }
    try {
      const long t = std::stol(line);
      if (t > max_temp) {
        max_temp = t;
      }
    }
    catch (const std::exception &) {
      continue;
    }
  }
  snap_.cpu_temperature = static_cast<float>(max_temp) / 1000.0f;
}

void HostStats::readCpuFreq() {
  std::ifstream online_file("/sys/devices/system/cpu/online");
  std::string   online_line;
  if (std::getline(online_file, online_line)) {
    const auto parts = splitByChar(online_line, '-');
    if (parts.size() >= 2) {
      try {
        cpu_cores_ = std::stoi(parts.at(1)) + 1;
      }
      catch (const std::exception &) {
        cpu_cores_ = 1;
      }
    }
  }

  long cpu_freq_khz = 0;
  for (int i = 0; i < cpu_cores_; ++i) {
    const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
    std::ifstream     file(path);
    std::string       line;
    if (!std::getline(file, line)) {
      continue;
    }
    try {
      cpu_freq_khz += std::stol(line);
    }
    catch (const std::exception &) {
      continue;
    }
  }

  if (cpu_cores_ > 0) {
    snap_.cpu_ghz = (static_cast<float>(cpu_freq_khz) / static_cast<float>(cpu_cores_)) / 1048576.0f;
  }
}

void HostStats::readMemLoad() {
  std::ifstream file("/proc/meminfo");
  std::string   line1, line2, line3, line4;
  if (!(std::getline(file, line1) && std::getline(file, line2) && std::getline(file, line3) && std::getline(file, line4))) {
    return;
  }

  auto parse_kib = [](const std::string &line) -> double {
    const auto results = splitByChar(line, ' ');
    for (size_t i = 1; i < results.size(); ++i) {
      if (!results[i].empty() && std::isdigit(results[i].front())) {
        try {
          return static_cast<double>(std::stol(results[i])) / 1048576.0; // KiB -> GiB
        }
        catch (const std::exception &) {
          return 0.0;
        }
      }
    }
    return 0.0;
  };

  const double total_ram = parse_kib(line1); // MemTotal
  const double free_ram  = parse_kib(line3); // MemAvailable (typically line 3 in modern kernels) — historical: MemFree
  const double buffers   = parse_kib(line4); // Buffers

  snap_.total_ram = static_cast<float>(total_ram);
  snap_.free_ram  = static_cast<float>(free_ram + buffers);
}

void HostStats::readDiskSpace() {
  try {
    const auto si  = std::filesystem::space(".");
    snap_.free_hdd = static_cast<int32_t>(std::round(static_cast<double>(si.available) / 1073741824.0));
  }
  catch (const std::exception &) {
    snap_.free_hdd = -1;
  }
}

void HostStats::readNodeCpuLoads() {
  snap_.node_cpu_loads.clear();

  // Need a valid system-wide tick denominator from the most recent readCpuLoad().
  if (cpu_total_diff_ <= 0 || cpu_cores_ <= 0) {
    return;
  }

  // Composable-node note: nodes sharing a container share one PID, so per-PID load
  // aggregates them. The __node:= label in readProcName() falls back to comm for
  // multi-node containers — fine for operator-facing "which process is hot" views.

  std::unordered_map<int, long> new_proc_ticks;
  std::unordered_set<int>       live_pids;

  std::error_code ec;
  // Iterate over /proc to find all live PIDs. For each, if it's a ROS process, read utime+stime and compare against the last snapshot to compute a load %.
  for (const auto &entry : std::filesystem::directory_iterator("/proc", ec)) {
    if (ec) {
      break;
    }
    const std::string name = entry.path().filename().string();
    if (name.empty() || !std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c); })) {
      continue;
    }
    int pid;
    try {
      pid = std::stoi(name);
    }
    catch (const std::exception &) {
      continue;
    }
    live_pids.insert(pid);

    // Cache the is-ROS decision per-PID to avoid re-scanning /proc/<pid>/maps every tick.
    auto cache_it = pid_is_ros_.find(pid);
    bool is_ros;
    if (cache_it != pid_is_ros_.end()) {
      is_ros = cache_it->second;
    } else {
      is_ros = isRosProcess(pid);
      pid_is_ros_.emplace(pid, is_ros);
    }
    if (!is_ros) {
      continue;
    }

    const long ticks = readProcTicks(pid);
    if (ticks < 0) {
      continue;
    }
    new_proc_ticks[pid] = ticks;

    // First time we see this PID we have no baseline — skip emitting until next tick.
    const auto last_it = proc_last_ticks_.find(pid);
    if (last_it == proc_last_ticks_.end()) {
      continue;
    }
    const long diff = ticks - last_it->second;
    if (diff <= 0) {
      continue;
    }
    // top-style %: 100% == one fully-saturated core; cpu_total_diff_ is aggregated over all cores.
    const float load_pct = 100.0f * static_cast<float>(diff) * static_cast<float>(cpu_cores_) / static_cast<float>(cpu_total_diff_);

    mrs_msgs::msg::CpuLoad msg;
    msg.node_name = readProcName(pid);
    msg.cpu_load  = load_pct;
    snap_.node_cpu_loads.push_back(std::move(msg));
  }

  proc_last_ticks_ = std::move(new_proc_ticks);

  // Prune the is-ROS cache for PIDs that no longer exist (guards against PID-reuse drift).
  for (auto it = pid_is_ros_.begin(); it != pid_is_ros_.end();) {
    if (live_pids.find(it->first) == live_pids.end()) {
      it = pid_is_ros_.erase(it);
    } else {
      ++it;
    }
  }
}

void HostStats::readWifi() {
  // Reset to "unavailable" sentinels; only populate on a successful read.
  snap_.wifi_interface.clear();
  snap_.wifi_signal_dbm   = -200.0f;
  snap_.wifi_link_quality = -1;

  std::ifstream file("/proc/net/wireless");
  if (!file.is_open()) {
    return;
  }

  std::string line;
  // Skip 2 header lines.
  std::getline(file, line);
  std::getline(file, line);

  while (std::getline(file, line)) {
    char  iface_buf[64] = {};
    int   status = 0, link = 0;
    float level = 0.0f, noise = 0.0f;

    if (std::sscanf(line.c_str(), "%63s %d %d. %f. %f.", iface_buf, &status, &link, &level, &noise) < 4) {
      continue;
    }

    std::string iface(iface_buf);
    if (!iface.empty() && iface.back() == ':') {
      iface.pop_back();
    }

    // If a specific interface is configured, only match that one.
    if (!wifi_interface_filter_.empty() && iface != wifi_interface_filter_) {
      continue;
    }

    snap_.wifi_interface    = std::move(iface);
    snap_.wifi_signal_dbm   = level;
    snap_.wifi_link_quality = link;
    return;
  }
}

} // namespace mrs_robot_diagnostics::utils
