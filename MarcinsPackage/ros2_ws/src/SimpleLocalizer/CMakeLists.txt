cmake_minimum_required(VERSION 3.5)
project(SimpleLocalizer)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)

# Create the executable
add_executable(simplelocalizer src/simplelocalizer.cpp)

# Specify the libraries to link against
target_link_libraries(simplelocalizer
  ${OpenCV_LIBS}
)

# Specify the C++ standard
set_target_properties(simplelocalizer PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Include directories
target_include_directories(simplelocalizer PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(simplelocalizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  OpenCV
  tf2
)

# Install the executable
install(TARGETS
simplelocalizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (if needed, e.g. launch files, config files)
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Package the project
ament_package()
