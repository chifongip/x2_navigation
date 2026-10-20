#ifndef X2_NAVIGATION__LOCALIZATION_GATE_HPP_
#define X2_NAVIGATION__LOCALIZATION_GATE_HPP_

#include <chrono>
#include <optional>

namespace x2_navigation
{

class LocalizationGate
{
public:
  using Clock = std::chrono::steady_clock;

  void update(bool ready, Clock::time_point now)
  {
    ready_ = ready;
    received_ = now;
  }

  bool ready(Clock::time_point now, std::chrono::duration<double> timeout) const
  {
    return ready_ && received_ && now >= *received_ && now - *received_ <= timeout;
  }

private:
  bool ready_{false};
  std::optional<Clock::time_point> received_;
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__LOCALIZATION_GATE_HPP_
