cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(servo_keyboard_input src/arm_controller.cpp)
target_include_directories(servo_keyboard_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo_keyboard_input PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(hand_status src/arm_controller.cpp)

add_executable(servo_joy_input src/arm_joy_controller.cpp)
target_include_directories(servo_joy_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo_joy_input PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(arm_information src/arm_information.cpp)
target_include_directories(arm_information PUBLIC include)
target_compile_features(arm_information PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  arm_information
  ament_cmake
  moveit
  rclcpp
  moveit_ros_planning_interface 
  moveit_ros_move_group
)

ament_target_dependencies(
  hand_status 
  ament_cmake
  moveit
  rclcpp
  std_msgs
  moveit_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  moveit_visual_tools
  tf2
  tf2_ros
)

ament_target_dependencies(
  servo_keyboard_input
  ament_cmake
  moveit
  rclcpp
  std_msgs
  moveit_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  moveit_visual_tools
  tf2
  tf2_ros
)

ament_target_dependencies(
  servo_joy_input
  ament_cmake
  moveit
  rclcpp
  std_msgs
  moveit_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  moveit_visual_tools
  tf2
  tf2_ros
)

install(
  TARGETS 
    servo_keyboard_input
    hand_status
    servo_joy_input
    arm_information
  DESTINATION 
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
