#ifndef X2_NAVIGATION__TIMING_HPP_
#define X2_NAVIGATION__TIMING_HPP_

#include <chrono>
#include <cmath>
#include <optional>

namespace x2_navigation
{

inline std::optional<std::chrono::nanoseconds> nanosecondsFromSeconds(double seconds)
{
  if (!std::isfinite(seconds)) {
    return std::nullopt;
  }

  // Keep one second of headroom so the floating-point conversion cannot round
  // into an out-of-range nanosecond value.
  const auto value = std::chrono::duration<double>(seconds);
  const auto minimum = std::chrono::duration<double>(
    std::chrono::nanoseconds::min() + std::chrono::seconds(1));
  const auto maximum = std::chrono::duration<double>(
    std::chrono::nanoseconds::max() - std::chrono::seconds(1));
  if (value < minimum || value > maximum) {
    return std::nullopt;
  }

  return std::chrono::duration_cast<std::chrono::nanoseconds>(value);
}

inline std::optional<std::chrono::nanoseconds> periodFromRateHz(double rate_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    return std::nullopt;
  }

  const auto period = nanosecondsFromSeconds(1.0 / rate_hz);
  if (!period || *period <= std::chrono::nanoseconds::zero()) {
    return std::nullopt;
  }
  return period;
}

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__TIMING_HPP_
