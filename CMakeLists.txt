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
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
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
    std_msgs 
    geometry_msgs 
    nav_msgs
    tf2
    tf2_ros
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
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui/actions
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui/dialogs
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui/panels
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui/widgets
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ros
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
)

# 添加源文件
file(GLOB_RECURSE UI_SOURCES 
  "src/ui/panels/*.cpp"
  "src/ui/widgets/*.cpp"
  "src/ui/actions/*.cpp"
  "src/ui/dialogs/*.cpp"
  "src/ui/*.cpp"
)

file(GLOB_RECURSE ROS_SOURCES
  "src/ros/*.cpp"
)

# 添加头文件
file(GLOB_RECURSE UI_HEADERS
  "include/ui/panels/*.h"
  "include/ui/widgets/*.h"
  "include/ui/actions/*.h"
  "include/ui/dialogs/*.h"
  "include/ui/*.h"
)

file(GLOB_RECURSE ROS_HEADERS
  "include/ros/*.h"
)

# 添加主程序源文件
list(APPEND UI_SOURCES "src/main.cpp")

# 创建可执行文件
add_executable(${PROJECT_NAME}
  ${UI_SOURCES}
  ${ROS_SOURCES}
  ${UI_HEADERS}
  ${ROS_HEADERS}
)

# 链接依赖库
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
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
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

