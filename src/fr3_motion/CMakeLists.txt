cmake_minimum_required(VERSION 3.8)
project(fr3_motion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_example_controllers REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  fr3_motion_controllers
  SHARED
  src/joint_impedance_controller.cpp
  src/cartesian_pose_controller.cpp
  src/joint_impedance_with_ik_controller.cpp
)

target_include_directories(fr3_motion_controllers
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(fr3_motion_controllers Franka::Franka)

ament_target_dependencies(
  fr3_motion_controllers
  controller_interface
  Franka
  franka_example_controllers
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface fr3_motion_controllers.xml)

install(
  TARGETS fr3_motion_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install controller configuration files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the commander example scripts.
install(PROGRAMS
  scripts/example_joint_impedance_commander
  scripts/example_cartesian_pose_commander
  scripts/example_joint_impedance_with_ik_commander
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Use the same linter set as franka_example_controllers (clang_format-based) instead of
  # ament_lint_auto's default ROS style (uncrustify/cpplint), since these controllers follow
  # the Google-style formatting of the code they are based on.
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_clang_format(CONFIG_FILE .clang-format include src)
  ament_copyright(include src config launch scripts test package.xml)
  ament_cppcheck(include src)
  ament_lint_cmake()
  ament_flake8()
  ament_pep257()
  ament_xmllint()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_joint_impedance_controller.launch_test.py TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_libraries(fr3_motion_controllers)
ament_export_dependencies(
  controller_interface
  franka_example_controllers
  franka_semantic_components
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
