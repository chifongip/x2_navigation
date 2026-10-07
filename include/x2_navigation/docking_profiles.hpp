#ifndef X2_NAVIGATION__DOCKING_PROFILES_HPP_
#define X2_NAVIGATION__DOCKING_PROFILES_HPP_

#include <charconv>
#include <cmath>
#include <limits>
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
  std::string detections_topic{"/front_center_rectify/detections"};
  std::string undock_mode{"tag_relative"};
  double timed_reverse_speed{0.1};
  double timed_reverse_duration{3.0};
  std::string target_source{"tag"};
};

// BoxState instance IDs encode the physical AprilTag, independently of box type.
inline DockingProfile bindBoxDockingProfile(
  DockingProfile profile, const std::string & instance_id, const std::string & tag_frame)
{
  if (profile.target_source != "box" || instance_id.rfind("tag:", 0) != 0 || tag_frame.empty()) {
    throw std::invalid_argument("box docking requires a tag:<id> instance");
  }
  const auto id_text = instance_id.substr(4);
  std::int64_t id = -1;
  const auto parsed = std::from_chars(id_text.data(), id_text.data() + id_text.size(), id);
  if (parsed.ec != std::errc{} || parsed.ptr != id_text.data() + id_text.size() || id < 0 ||
    id > std::numeric_limits<std::int32_t>::max() || std::to_string(id) != id_text)
  {
    throw std::invalid_argument("invalid box tag instance: " + instance_id);
  }
  profile.tag_id = id;
  profile.tag_frame = tag_frame;
  return profile;
}

class DockingProfiles
{
public:
  void add(const DockingProfile & profile)
  {
    if (profile.name.empty() || profile.name.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos ||
      (profile.target_source != "tag" && profile.target_source != "box") ||
      (profile.target_source == "tag" && (profile.tag_id < 0 || profile.tag_frame.empty())) ||
      profile.detections_topic.empty() ||
      (profile.undock_mode != "tag_relative" && profile.undock_mode != "timed_reverse") ||
      !std::isfinite(profile.timed_reverse_speed) || profile.timed_reverse_speed <= 0.0 ||
      profile.timed_reverse_speed > 0.5 || !std::isfinite(profile.timed_reverse_duration) ||
      profile.timed_reverse_duration <= 0.0 ||
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
