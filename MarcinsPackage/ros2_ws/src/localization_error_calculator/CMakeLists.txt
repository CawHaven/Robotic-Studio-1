cmake_minimum_required(VERSION 3.5)
project(localization_error_calculator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Add executable
add_executable(localization_error_calculator src/errorcalc.cpp)

# Link libraries
ament_target_dependencies(localization_error_calculator
  rclcpp
  geometry_msgs
  gazebo_msgs
)

# Install executable
install(TARGETS
  localization_error_calculator
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()
