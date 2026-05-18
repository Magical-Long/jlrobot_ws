cmake_minimum_required(VERSION 3.5)
project(robot_description)

# 这里虽然包内没有主要的 C/C++ 逻辑，
# 仍保持与工作区其他 ROS2 包一致的基础编译配置。
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

# `robot_description` 主要是资源包：
# - 安装 URDF / mesh / launch / config
# - 供其他包在运行时通过 ament index 查找
# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
