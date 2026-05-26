#pragma once

#include <cstddef>
#include <deque>

#include <rclcpp/rclcpp.hpp>

namespace mrs_robot_diagnostics::utils
{

/**
 * @brief Sliding-window message-rate tracker.
 *
 * Records arrival timestamps and computes the average rate over the most recent
 * window. Not thread-safe — callers are expected to synchronise externally if
 * the record() and rate() calls can race.
 */
class RateTracker {
public:
  explicit RateTracker(std::size_t window_size = 10)
      : window_size_(window_size) {
  }

  /** @brief Record a new message arrival at @p t. */
  void record(const rclcpp::Time &t) {
    timestamps_.push_back(t);
    if (timestamps_.size() > window_size_) {
      timestamps_.pop_front();
    }
  }

  /** @brief Average rate over the current window, or 0.0 if insufficient samples. */
  double rate() const {
    if (timestamps_.size() < 2) {
      return 0.0;
    }
    const double span = (timestamps_.back() - timestamps_.front()).seconds();
    if (span <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(timestamps_.size() - 1) / span;
  }

  void clear() {
    timestamps_.clear();
  }

private:
  std::size_t              window_size_;
  std::deque<rclcpp::Time> timestamps_;
};

} // namespace mrs_robot_diagnostics::utils
