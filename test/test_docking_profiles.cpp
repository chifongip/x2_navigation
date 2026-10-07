#include "x2_navigation/docking_profiles.hpp"

#include <limits>

#include <gtest/gtest.h>

using x2_navigation::DockingProfile;
using x2_navigation::DockingProfiles;

TEST(DockingProfiles, SelectsDefaultRememberedAndExplicitProfiles)
{
  DockingProfiles profiles;
  profiles.add({"default", 9, "tag9", 0.5, 0.0, 0.0});
  profiles.add({"offset", 9, "tag9", 0.7, -0.1, 0.2});
  profiles.add({"other", 10, "tag10", 0.6, 0.0, 0.0});
  EXPECT_EQ(profiles.resolve("", "default").name, "default");
  EXPECT_EQ(profiles.resolve("", "offset").name, "offset");
  EXPECT_EQ(profiles.resolve("", "default", "offset").name, "offset");
  EXPECT_EQ(profiles.resolve("other", "default", "offset").tag_id, 10);
  EXPECT_DOUBLE_EQ(profiles.resolve("offset", "default").lateral_offset, -0.1);
  EXPECT_THROW(profiles.resolve("missing", "default"), std::invalid_argument);
  EXPECT_THROW(profiles.resolve("", "missing"), std::invalid_argument);
}

TEST(DockingProfiles, RejectsInvalidAndDuplicateConfigurations)
{
  const DockingProfile valid{"default", 9, "tag9", 0.5, 0.0, 0.0};
  DockingProfiles profiles;
  profiles.add(valid);
  EXPECT_THROW(profiles.add(valid), std::invalid_argument);
  for (const auto & name : {"", "bad.name", "bad/name"}) {
    auto profile = valid;
    profile.name = name;
    EXPECT_THROW(profiles.add(profile), std::invalid_argument);
  }
  auto profile = valid;
  profile.name = "new";
  profile.tag_id = -1;
  EXPECT_THROW(profiles.add(profile), std::invalid_argument);
  profile = valid;
  profile.name = "new";
  profile.tag_frame.clear();
  EXPECT_THROW(profiles.add(profile), std::invalid_argument);
  for (const double value : {0.0, -0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    profile = valid;
    profile.name = "new";
    profile.standoff = value;
    EXPECT_THROW(profiles.add(profile), std::invalid_argument);
  }
  profile = valid;
  profile.name = "new";
  profile.lateral_offset = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(profiles.add(profile), std::invalid_argument);
  profile.lateral_offset = 0.0;
  profile.yaw_offset = std::numeric_limits<double>::infinity();
  EXPECT_THROW(profiles.add(profile), std::invalid_argument);
}

TEST(DockingProfiles, SupportsTimedBoxProfileAndRejectsInvalidSettings)
{
  DockingProfile box{"grey_box", 180, "tag180", 0.5, 0.0, 0.0,
    "/detections", "timed_reverse", 0.1, 3.0};
  DockingProfiles profiles;
  profiles.add(box);
  const auto & selected = profiles.resolve("", "default", "grey_box");
  EXPECT_EQ(selected.detections_topic, "/detections");
  EXPECT_EQ(selected.undock_mode, "timed_reverse");
  for (const double value : {0.0, -0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto invalid = box;
    invalid.timed_reverse_duration = value;
    EXPECT_THROW(DockingProfiles{}.add(invalid), std::invalid_argument);
    invalid = box;
    invalid.timed_reverse_speed = value;
    EXPECT_THROW(DockingProfiles{}.add(invalid), std::invalid_argument);
  }
  box.timed_reverse_speed = 0.51;
  EXPECT_THROW(DockingProfiles{}.add(box), std::invalid_argument);
  box.timed_reverse_speed = 0.1;
  box.undock_mode = "fallback";
  EXPECT_THROW(DockingProfiles{}.add(box), std::invalid_argument);
  box.undock_mode = "tag_relative";
  box.detections_topic.clear();
  EXPECT_THROW(DockingProfiles{}.add(box), std::invalid_argument);
}

TEST(DockingProfiles, BindsAnyBoxInstanceWithoutChangingTemplate)
{
  x2_navigation::DockingProfile box{"box", -1, "", 0.5, 0.0, 0.0,
    "/detections", "timed_reverse", 0.1, 3.0, "box"};
  DockingProfiles profiles;
  profiles.add(box);
  for (const auto & instance : {"tag:0", "tag:17", "tag:180"}) {
    const auto bound = x2_navigation::bindBoxDockingProfile(profiles.resolve("box", "default"), instance, "marker_" + std::string(instance).substr(4));
    EXPECT_EQ(bound.tag_frame, "marker_" + std::string(instance).substr(4));
    EXPECT_GE(bound.tag_id, 0);
    EXPECT_EQ(bound.name, "box");
  }
  EXPECT_EQ(profiles.resolve("box", "default").tag_id, -1);
  for (const auto & invalid : {"", "legacy", "tag:-1", "tag:01", "tag:1x", "tag:2147483648"}) {
    EXPECT_THROW(x2_navigation::bindBoxDockingProfile(box, invalid, "tag17"), std::invalid_argument);
  }
}
