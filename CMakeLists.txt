# 基本配置
cmake_minimum_required(VERSION 3.0.2)
project(robot_control_gui)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加调试标志
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

# OpenGL相关设置
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
add_definitions(-DQT_NO_OPENGL_ES_2)
add_definitions(-DFORCE_DESKTOP_OPENGL)
add_definitions(-DRVIZ_OPENGL_VERSION=21)  # 强制使用OpenGL 2.1
add_definitions(-DRVIZ_RENDERING_IMPLEMENTATION=RVIZ_RENDERING_IMPLEMENTATION_OGRE)
add_definitions(-DOGRE_GLSUPPORT_USE_EGL=0)

# Qt相关设置
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  move_base_msgs
  actionlib
  tf2
  tf2_ros
  cv_bridge
  image_transport
  rviz
)

# 查找Qt5
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
  Network
  Qml
  Script
  SerialPort
  OpenGL
)

# 配置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    std_srvs
    geometry_msgs 
    nav_msgs 
    sensor_msgs
    diagnostic_msgs
    move_base_msgs
    actionlib
    tf2
    tf2_ros
    cv_bridge
    image_transport
    rviz
  DEPENDS 
    Qt5Core 
    Qt5Widgets 
    Qt5Gui 
    Qt5Network
    Qt5Qml
    Qt5Script
    Qt5SerialPort
    OPENGL
    GLUT
)

# 设置包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Script_INCLUDE_DIRS}
  ${Qt5SerialPort_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# 添加Qt编译标志
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

# 添加编译选项
add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-DQT_NO_KEYWORDS)
add_compile_options(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# 添加Qt关键字支持
remove_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DQT_WIDGETS_LIB)
add_definitions(-DQT_GUI_LIB)
add_definitions(-DQT_CORE_LIB)

# 设置Qt的MOC选项
set(CMAKE_AUTOMOC_MOC_OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED")

# 添加tf2依赖
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 收集源文件
file(GLOB_RECURSE SOURCE_FILES
  src/*.cpp
)

# 收集头文件
file(GLOB_RECURSE HEADER_FILES
  include/*.h
)

# 创建主程序库
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

# 添加依赖关系
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接主程序库的依赖
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::Network
  Qt5::Qml
  Qt5::Script
  Qt5::SerialPort
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# 创建主程序可执行文件
add_executable(${PROJECT_NAME}_node src/main.cpp)

# 添加可执行文件的依赖
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接可执行文件
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装规则
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

