cmake_minimum_required(VERSION 3.22)
project(x2_navigation)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(point_cloud_voxel_filter STATIC src/point_cloud_voxel_filter.cpp)
target_compile_features(point_cloud_voxel_filter PUBLIC cxx_std_17)
target_compile_options(point_cloud_voxel_filter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(point_cloud_voxel_filter PUBLIC include)
ament_target_dependencies(point_cloud_voxel_filter
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(fast_lio_odom_adapter src/fast_lio_odom_adapter.cpp)
target_compile_features(fast_lio_odom_adapter PUBLIC cxx_std_17)
target_compile_options(fast_lio_odom_adapter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fast_lio_odom_adapter PRIVATE include)
target_link_libraries(fast_lio_odom_adapter Eigen3::Eigen)
ament_target_dependencies(fast_lio_odom_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

add_executable(nav2_zmq_velocity_bridge src/nav2_zmq_velocity_bridge.cpp)
target_compile_features(nav2_zmq_velocity_bridge PUBLIC cxx_std_17)
target_compile_options(nav2_zmq_velocity_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(nav2_zmq_velocity_bridge PRIVATE include)
target_link_libraries(nav2_zmq_velocity_bridge PkgConfig::ZMQ)
ament_target_dependencies(nav2_zmq_velocity_bridge
  geometry_msgs
  rclcpp
)

add_executable(lidar_cloud_throttle src/lidar_cloud_throttle.cpp)
target_compile_features(lidar_cloud_throttle PUBLIC cxx_std_17)
target_compile_options(lidar_cloud_throttle PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(lidar_cloud_throttle PRIVATE include)
target_link_libraries(lidar_cloud_throttle point_cloud_voxel_filter)
ament_target_dependencies(lidar_cloud_throttle
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  fast_lio_odom_adapter
  nav2_zmq_velocity_bridge
  lidar_cloud_throttle
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch map rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_odometry_transform test/test_odometry_transform.cpp)
  target_compile_options(test_odometry_transform PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_odometry_transform PRIVATE include)
  target_link_libraries(test_odometry_transform Eigen3::Eigen)
  ament_target_dependencies(test_odometry_transform
    geometry_msgs
    nav_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
  )

  ament_add_gtest(test_velocity_command test/test_velocity_command.cpp)
  target_compile_options(test_velocity_command PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_velocity_command PRIVATE include)
  ament_target_dependencies(test_velocity_command geometry_msgs)

  ament_add_gtest(test_point_cloud_voxel_filter test/test_point_cloud_voxel_filter.cpp)
  target_compile_options(test_point_cloud_voxel_filter PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_point_cloud_voxel_filter point_cloud_voxel_filter)
  ament_target_dependencies(test_point_cloud_voxel_filter
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
  )

  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)
  add_launch_test(test/test_fast_lio_odom_adapter.launch.py TIMEOUT 30)
  add_launch_test(test/test_navigation.launch.py TIMEOUT 30)
  add_launch_test(test/test_nav2_zmq_velocity_bridge.launch.py TIMEOUT 30)
endif()

ament_package()
