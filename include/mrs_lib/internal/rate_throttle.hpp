#ifndef MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_
#define MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace mrs_lib::internal
{

  /**
   * @brief Decides which messages of a stream to pass so that the output rate does not exceed a cap.
   *
   * Deadline (GCRA-like) scheduling: the next allowed time advances by one period per passed message,
   * so a faster, jittery input is decimated to the cap on average instead of undershooting it.
   * A message up to period/4 early is passed, a late message keeps at most period/2 of catch-up
   * credit, so there are no bursts after a pause (passed messages are at least period/4 apart).
   * An input slower than the cap is passed as is. Not thread-safe.
   */
  class RateThrottle
  {
  public:
    /**
     * @param rate maximum output rate [Hz], <= 0 disables throttling
     */
    explicit RateThrottle(const double rate) : period_ns_(rate > 0 ? std::llround(std::min(1e9 / rate, double(max_period_ns_))) : 0)
    {
    }

    /**
     * @param period minimum output period, <= 0 disables throttling
     */
    explicit RateThrottle(const std::chrono::nanoseconds period) : period_ns_(std::clamp<int64_t>(period.count(), 0, max_period_ns_))
    {
    }

    /**
     * @brief decide whether a message arriving at \p now_ns should be passed (and register it if so)
     *
     * @param now_ns current time [ns]
     *
     * @return true if the message should be passed
     */
    bool accept(const int64_t now_ns)
    {
      if (period_ns_ <= 0)
      {
        return true;
      }

      // first message, or the clock jumped backwards (e.g., sim time reset)
      if (!next_ns_ || now_ns < *next_ns_ - (3 * period_ns_) / 2)
      {
        next_ns_ = now_ns;
      }

      if (now_ns < *next_ns_ - period_ns_ / 4)
      {
        return false;
      }

      next_ns_ = std::max(*next_ns_ + period_ns_, now_ns + period_ns_ / 2);

      return true;
    }

  private:
    // keeps the deadline arithmetic far from int64 overflow (~73 years)
    static constexpr int64_t max_period_ns_ = std::numeric_limits<int64_t>::max() / 4;

    int64_t period_ns_;

    std::optional<int64_t> next_ns_;
  };

} // namespace mrs_lib::internal

#endif // MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_
