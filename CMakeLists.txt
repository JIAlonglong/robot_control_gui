cmake_minimum_required(VERSION 3.0.2)
project(robot_control_gui)

# 使用C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  tf2
  map_server
  rviz
)

# 查找Qt5
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)

# 配置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs move_base_msgs actionlib tf2 map_server rviz
  DEPENDS Qt5Core Qt5Widgets
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 启用Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 收集源文件（排除测试文件）
file(GLOB_RECURSE SOURCES
  "src/ui/*.cpp"
  "src/ros/*.cpp"
  "src/main.cpp"
)

# 收集头文件
file(GLOB_RECURSE HEADERS
  "include/ui/*.h"
  "include/ros/*.h"
)

# 创建库目标
add_library(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
)

# 链接依赖
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_app src/main.cpp)
target_link_libraries(${PROJECT_NAME}_app
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

# 安装规则
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

