#ifndef X2_NAVIGATION__DOCKING_PROFILES_HPP_
#define X2_NAVIGATION__DOCKING_PROFILES_HPP_

#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

namespace x2_navigation
{

struct DockingProfile
{
  std::string name;
  std::int64_t tag_id;
  std::string tag_frame;
  double standoff;
  double lateral_offset;
  double yaw_offset;
};

class DockingProfiles
{
public:
  void add(const DockingProfile & profile)
  {
    if (profile.name.empty() || profile.name.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos ||
      profile.tag_id < 0 || profile.tag_frame.empty() ||
      !std::isfinite(profile.standoff) || profile.standoff <= 0.0 ||
      !std::isfinite(profile.lateral_offset) || !std::isfinite(profile.yaw_offset))
    {
      throw std::invalid_argument("invalid docking profile: " + profile.name);
    }
    if (!profiles_.emplace(profile.name, profile).second) {
      throw std::invalid_argument("duplicate docking profile: " + profile.name);
    }
  }

  const DockingProfile & resolve(
    const std::string & requested, const std::string & default_profile,
    const std::string & last_docked = "") const
  {
    const auto & name = requested.empty() ?
      (last_docked.empty() ? default_profile : last_docked) : requested;
    const auto found = profiles_.find(name);
    if (found == profiles_.end()) {
      throw std::invalid_argument("unknown docking profile: " + name);
    }
    return found->second;
  }

private:
  std::map<std::string, DockingProfile> profiles_;
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__DOCKING_PROFILES_HPP_
