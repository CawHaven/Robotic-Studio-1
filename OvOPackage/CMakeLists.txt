cmake_minimum_required(VERSION 3.8)
project(robotics_studio_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executables
# add_executable(sprint1 src/sprint1.cpp)
# ament_target_dependencies(sprint1 rclcpp std_msgs sensor_msgs nav_msgs)
# target_link_libraries(sprint1 ${OpenCV_LIBS})

# add_executable(sprint2 src/sprint2.cpp)
# ament_target_dependencies(sprint2 rclcpp std_msgs sensor_msgs nav_msgs)
# target_link_libraries(sprint2 ${OpenCV_LIBS})

# add_executable(sprint3 src/sprint3.cpp)
# ament_target_dependencies(sprint3 rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
# target_link_libraries(sprint3 ${OpenCV_LIBS})


# add_executable(sprint3part2 src/sprint3part2.cpp)
# ament_target_dependencies(sprint3part2 rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
# target_link_libraries(sprint3part2 ${OpenCV_LIBS} yaml-cpp)

add_executable(sprint4 src/sprint4.cpp)
ament_target_dependencies(sprint4 rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(sprint4 ${OpenCV_LIBS} yaml-cpp)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyrights (if needed)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  # sprint1
  # sprint2
  # sprint3
  # sprint3part2
  sprint4
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
