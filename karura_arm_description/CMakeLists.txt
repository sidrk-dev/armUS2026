cmake_minimum_required(VERSION 3.5)
project(karura_arm_description)

# If you really need a specific C++ standard:
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the necessary ROS 2 packages (if you actually need them)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

# Install all the resource directories (URDF, meshes, etc.) into share/<package_name>/
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Replace catkin_package() with ament_package()
ament_package()
