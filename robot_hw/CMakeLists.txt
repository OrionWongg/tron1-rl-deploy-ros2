cmake_minimum_required(VERSION 3.8)
project(robot_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译优化选项
add_compile_options(-O3)

# --- 查找所有必要的 ROS2 包和外部库 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED) # 自定义消息
find_package(websocketpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED CONFIG) # 使用 CONFIG 模式查找，推荐
find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(limxsdk_lowlevel REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED) # 实时控制工具

# ICP节点新增的依赖
find_package(sensor_msgs REQUIRED) # LaserScan, PointCloud2
find_package(pcl_conversions REQUIRED) # ROS PointCloud2 <-> PCL conversions
find_package(laser_geometry REQUIRED) # LaserScan to PointCloud2 projection
find_package(PCL REQUIRED) # Point Cloud Library for ICP

# --- 添加包含目录 ---
include_directories(
  include
  # include/${PROJECT_NAME}  # 如果您的头文件在 include/robot_hw/ 目录下，则需要
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} # PCL 头文件目录
)

# --- 定义可执行文件和链接库 ---

## object_detection_node 节点
add_executable(tron_ctrl_node src/TronControl.cpp src/RobotControlInterface.cpp)
ament_target_dependencies(tron_ctrl_node
  rclcpp
  std_msgs
  nav_msgs
  robot_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(tron_ctrl_node
  Boost::system # 明确 Boost 的 system 组件
  Boost::thread # 明确 Boost 的 thread 组件
  websocketpp::websocketpp # 如果 websocketpp 提供目标
  nlohmann_json::nlohmann_json
  OpenSSL::SSL # 明确 OpenSSL SSL 库
  OpenSSL::Crypto # 明确 OpenSSL Crypto 库
)

## pointfoot_node 节点
add_executable(pointfoot_node
  src/HardwareBase.cpp
  src/PointfootHardware.cpp
  src/HardwareLoop.cpp
  src/PointfootHardwareNode.cpp
)
ament_target_dependencies(pointfoot_node
  rclcpp
  hardware_interface
  controller_manager
  controller_manager_msgs
  urdf
  limxsdk_lowlevel
  realtime_tools # 如果使用实时工具
)


# --- 安装可执行目标 ---
install(TARGETS
  tron_ctrl_node
  pointfoot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()