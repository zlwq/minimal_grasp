cmake_minimum_required(VERSION 3.4)
project(franka_example_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
)

catkin_package()

## 安装你需要的脚本
catkin_install_python(
  PROGRAMS
    scripts/interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装你需要的 rviz 配置文件
install(FILES
  launch/rviz/franka_dual_description_with_marker.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## 如果还要保留 launch 目录整体（方便调用）
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
