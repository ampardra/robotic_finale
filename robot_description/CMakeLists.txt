cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# For custom service
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# ---- Service generation ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

add_executable(frame_id_converter_node scripts/frame_id_converter.cpp)
ament_target_dependencies(frame_id_converter_node rclcpp sensor_msgs)

add_executable(motor_command_node scripts/motor_command.cpp)
ament_target_dependencies(motor_command_node rclcpp std_msgs)

add_executable(ekf_diff_imu_node scripts/ekf_diff_imu.cpp)
target_include_directories(ekf_diff_imu_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ekf_diff_imu_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
)

# ---- A* planner node ----
add_executable(astar_planner_node src/astar_planner_node.cpp)
ament_target_dependencies(astar_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pid_path_follower_node src/pid_path_follower_node.cpp)
ament_target_dependencies(pid_path_follower_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

add_executable(mpc_path_follower_node src/mpc_path_follower_node.cpp)
ament_target_dependencies(mpc_path_follower_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)



# Link generated service typesupport to this target
rosidl_target_interfaces(astar_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY src launch rviz config world scripts maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    frame_id_converter_node
    ekf_diff_imu_node
    motor_command_node
    astar_planner_node
    pid_path_follower_node
    mpc_path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
