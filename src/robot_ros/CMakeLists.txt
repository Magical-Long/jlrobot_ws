cmake_minimum_required(VERSION 3.5)
project(robot_ros)

# 保持与其他包一致的基础语言标准设置。
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 这个包主要充当 `robot_core` 和 ROS 可视化链路之间的桥接层。
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_core REQUIRED)
find_package(robot_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# `joint_state_demo` 节点负责：
# - 生成示例轨迹
# - 发布 `/joint_states`
# - 让 RViz / robot_state_publisher 可直接观察结果
add_executable(joint_state_demo
  src/joint_state_demo.cc
  src/joint_state_demo_main.cc
)

target_include_directories(joint_state_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joint_state_demo
  ament_index_cpp
  rclcpp
  robot_core
  robot_utils
  sensor_msgs
)

# `gazebo_pose_controller` 节点负责：
# - 订阅目标末端位姿
# - 调用 robot_core 做 IK 和轨迹插值
# - 发布关节轨迹到 Gazebo 的 arm_controller
add_executable(gazebo_pose_controller
  src/gazebo_pose_controller.cc
  src/gazebo_pose_controller_main.cc
)

target_include_directories(gazebo_pose_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gazebo_pose_controller
  ament_index_cpp
  geometry_msgs
  rclcpp
  robot_core
  robot_utils
  sensor_msgs
  trajectory_msgs
)

# `gazebo_effort_controller` 节点负责：
# - 订阅 Gazebo 当前 joint_states
# - 订阅目标关节参考
# - 调用 robot_core 的关节空间 PD 力矩控制器
# - 发布力矩命令到 effort controller
add_executable(gazebo_effort_controller
  src/gazebo_effort_controller.cc
  src/gazebo_effort_controller_main.cc
)

target_include_directories(gazebo_effort_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gazebo_effort_controller
  ament_index_cpp
  rclcpp
  robot_core
  robot_utils
  sensor_msgs
  std_msgs
)

# `gazebo_pose_effort_controller` 节点负责：
# - 订阅目标末端位姿
# - 调用 robot_core 做 IK 和关节轨迹插值
# - 发布 JointState 参考给下层 PD 力矩控制器
add_executable(gazebo_pose_effort_controller
  src/gazebo_pose_effort_controller.cc
  src/gazebo_pose_effort_controller_main.cc
)

target_include_directories(gazebo_pose_effort_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gazebo_pose_effort_controller
  ament_index_cpp
  geometry_msgs
  rclcpp
  robot_core
  robot_utils
  sensor_msgs
)

# `gazebo_admittance_controller` 节点负责：
# - 订阅原始目标位姿和 FT 传感器 wrench
# - 用最小三维导纳外环把外力转换成位姿偏移
# - 发布修正后的目标位姿给现有 pose-to-effort 内环
add_executable(gazebo_admittance_controller
  src/gazebo_admittance_controller.cc
  src/gazebo_admittance_controller_main.cc
)

target_include_directories(gazebo_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gazebo_admittance_controller
  Eigen3::Eigen
)

ament_target_dependencies(gazebo_admittance_controller
  geometry_msgs
  rclcpp
  robot_utils
)

# 安装头文件与运行时资源，供其他包复用或直接启动。
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    scripts/keyboard_command_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_command_sender
)

install(
  PROGRAMS
    scripts/ee_force_tester.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ee_force_tester
)

# 清理旧安装布局遗留在 `share/${PROJECT_NAME}` 根目录下的 launch / yaml 软链接，
# 避免 `ros2 launch` 在新旧两套路径里同时发现同名文件。
install(CODE
  "
  file(GLOB _robot_ros_legacy_launch_files
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/*.launch.py\")
  file(GLOB _robot_ros_legacy_yaml_files
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/*.yaml\")
  if(_robot_ros_legacy_launch_files)
    file(REMOVE \${_robot_ros_legacy_launch_files})
  endif()
  if(_robot_ros_legacy_yaml_files)
    file(REMOVE \${_robot_ros_legacy_yaml_files})
  endif()
  "
)

install(
  TARGETS
    joint_state_demo
    gazebo_pose_controller
    gazebo_effort_controller
    gazebo_pose_effort_controller
    gazebo_admittance_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
