cmake_minimum_required(VERSION 3.0.2)
project(robot_control_gui)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
  rviz
  tf2
  tf2_ros
  move_base_msgs
  actionlib
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
)

find_package(OpenCV REQUIRED)

# 设置OGRE的查找路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/lib/OGRE/cmake")

# 启用Qt的MOC、UIC和RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 设置包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/OGRE
)

# 设置Qt的定义
add_definitions(${Qt5Widgets_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs geometry_msgs cv_bridge image_transport rviz tf2 tf2_ros move_base_msgs actionlib
  DEPENDS Qt5Core Qt5Widgets Qt5Gui OpenCV
)

# 收集所有源文件
file(GLOB_RECURSE QT_SOURCES 
  src/ui/*.cpp
  src/ros/*.cpp
)

# 收集所有头文件
file(GLOB_RECURSE QT_HEADERS
  include/ui/*.h
  include/ros/*.h
)

# 创建库目标
add_library(${PROJECT_NAME}
  ${QT_SOURCES}
  ${QT_HEADERS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${OpenCV_LIBRARIES}
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

# 安装launch文件
install(DIRECTORY src/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装资源文件
install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  FILES_MATCHING PATTERN "*.png"
  PATTERN "*.jpg"
)

