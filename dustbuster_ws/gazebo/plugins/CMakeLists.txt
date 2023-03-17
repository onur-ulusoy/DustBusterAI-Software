cmake_minimum_required(VERSION 3.8)
project(db_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(/opt/ros/humble/include/geometry_msgs/)
include_directories(/opt/ros/humble/include)
include_directories(/home/onur/ws/src/gazebo_ros_pkgs/gazebo_ros/include)

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

target_include_directories(wheel_velocity_setter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wheel_velocity_setter
  ${GAZEBO_LIBRARIES}
  ${ament_LIBRARIES}
)
target_compile_features(wheel_velocity_setter PUBLIC cxx_std_17)
ament_target_dependencies(
  wheel_velocity_setter
  gazebo_ros_pkgs
  rclcpp
  std_msgs
)

# Install the shared library
install(TARGETS wheel_velocity_setter
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
