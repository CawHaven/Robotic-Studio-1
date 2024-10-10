cmake_minimum_required(VERSION 3.5)
project(Sprint3)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the executable for the cylinder detection node
add_executable(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  gazebo_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  OpenCV
)

# Declare the executable for the map overlay node
add_executable(map_overlay src/map_overlay.cpp)
ament_target_dependencies(map_overlay 
  rclcpp 
  nav_msgs 
  OpenCV
)

# Install targets
install(TARGETS
  cylinder_detector
  map_overlay
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
