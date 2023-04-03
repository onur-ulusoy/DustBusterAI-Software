cmake_minimum_required(VERSION 3.8)
project(db_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Add Gazebo and SDF headers
find_package(gazebo REQUIRED)
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create a shared library from wheel_velocity_setter.cpp
add_library(wheel_velocity_setter SHARED src/wheel_velocity_setter.cpp)
# Create a shared library from diff_drive_wheel_encoder.cpp
add_library(diff_drive_wheel_encoder SHARED src/diff_drive_wheel_encoder.cpp)

target_include_directories(wheel_velocity_setter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(diff_drive_wheel_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wheel_velocity_setter
  ${GAZEBO_LIBRARIES}
  ${ament_LIBRARIES}
)
target_link_libraries(diff_drive_wheel_encoder
  ${GAZEBO_LIBRARIES}
  ${ament_LIBRARIES}
)
target_compile_features(wheel_velocity_setter PUBLIC cxx_std_17)
target_compile_features(diff_drive_wheel_encoder PUBLIC cxx_std_17)
ament_target_dependencies(
  wheel_velocity_setter
  gazebo_ros_pkgs
  rclcpp
  tf2_ros
  nav_msgs
  geometry_msgs
)
ament_target_dependencies(
  diff_drive_wheel_encoder
  gazebo_ros_pkgs
  rclcpp
  tf2_ros
  nav_msgs
  geometry_msgs
)

# Install the shared library
install(TARGETS wheel_velocity_setter
  diff_drive_wheel_encoder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
