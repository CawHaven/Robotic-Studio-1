cmake_minimum_required(VERSION 3.5)
project(scan_sub_pub)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(scan_sub_pub_node src/scanSubPub.cpp)

# Include directories
target_include_directories(scan_sub_pub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(scan_sub_pub_node rclcpp sensor_msgs)

# Install the executable
install(TARGETS scan_sub_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
