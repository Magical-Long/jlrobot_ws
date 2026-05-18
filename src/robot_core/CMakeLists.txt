cmake_minimum_required(VERSION 3.5)
project(robot_core)

# `robot_core` 是整个工作区里放算法和模型封装的核心包，
# 因此这里统一声明基础语言标准和常用依赖。
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 共享库 `robot_core` 集中承载：
# - RobotModel
# - IK 求解
# - 势场法避障
# - MoveIt 避障
# - 轨迹插值
# - 实验组织逻辑
add_library(robot_core SHARED
  src/filters.cc
  src/joint_space_pd_controller.cc
  src/experiment_examples.cc
  src/robot_model.cc
  src/ik_solver.cc
  src/moveit_obstacle_avoidance_planner.cc
  src/obstacle_avoidance_planner.cc
  src/trajectory_planner.cc
)

target_include_directories(robot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_core
  Eigen3::Eigen
  pinocchio::pinocchio
)

ament_target_dependencies(robot_core
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  sensor_msgs
  shape_msgs
  trajectory_msgs
)

# 这个 demo 节点把 YAML / launch 参数和核心算法库串起来，
# 便于直接做实验复现与日志对照。
add_executable(experiment_examples_demo_node
  src/experiment_examples_demo_node.cc
)

target_link_libraries(experiment_examples_demo_node
  robot_core
  Eigen3::Eigen
  pinocchio::pinocchio
)

ament_target_dependencies(experiment_examples_demo_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
)

# 安装头文件、launch 和配置文件，供运行时通过 ament index 查找。
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS robot_core
  EXPORT robot_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS experiment_examples_demo_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(robot_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  Eigen3
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pinocchio
  rclcpp
  sensor_msgs
  shape_msgs
  trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
