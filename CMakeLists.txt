# 基本配置
cmake_minimum_required(VERSION 3.0.2)
project(robot_control_gui)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加调试标志
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 启用Qt自动生成
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 设置Qt5路径
set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")

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
  dynamic_reconfigure
  interactive_markers
  cv_bridge
  image_transport
  rviz
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
  OpenGL
  Network
  PrintSupport
  Multimedia
  MultimediaWidgets
  SerialPort
)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 设置OGRE的查找路径
if(NOT DEFINED OGRE_PREFIX_DIR)
  set(OGRE_PREFIX_DIR "/usr")
endif()

set(OGRE_INCLUDE_DIRS 
  ${OGRE_PREFIX_DIR}/include/OGRE
  ${OGRE_PREFIX_DIR}/include/OGRE/Overlay
)

set(OGRE_LIBRARIES
  OgreMain
  OgreOverlay
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
    Qt5OpenGL 
    Qt5Network 
    Qt5PrintSupport 
    Qt5Multimedia 
    Qt5MultimediaWidgets 
    Qt5SerialPort 
    OpenCV
    OGRE
)

# 添加Qt5编译标志
add_definitions(-DQT_WIDGETS_LIB -DQT_GUI_LIB -DQT_CORE_LIB)

# 设置包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${Qt5Multimedia_INCLUDE_DIRS}
  ${Qt5MultimediaWidgets_INCLUDE_DIRS}
  ${Qt5SerialPort_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  /usr/include/x86_64-linux-gnu/qt5
  /usr/include/x86_64-linux-gnu/qt5/QtWidgets
  /usr/include/x86_64-linux-gnu/qt5/QtGui
  /usr/include/x86_64-linux-gnu/qt5/QtCore
)

# 收集源文件
file(GLOB_RECURSE QT_SOURCES
  src/ui/*.cpp
  src/ros/*.cpp
)

file(GLOB_RECURSE QT_HEADERS
  include/ui/*.h
  include/ros/*.h
)

# 创建主程序库
add_library(${PROJECT_NAME} SHARED
  ${QT_SOURCES}
  ${QT_HEADERS}
  src/ros/robot_controller.cpp
  src/ui/main_window.cpp
  src/ui/navigation_panel.cpp
  src/ui/control_panel.cpp
  src/ui/settings_panel.cpp
  src/ui/dashboard_window.cpp
  src/ui/rviz_view.cpp
  src/ui/planner_settings_dialog.cpp
  src/ui/goal_setting_dialog.cpp
)

# 链接主程序库的依赖
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Network
  Qt5::PrintSupport
  Qt5::Multimedia
  Qt5::MultimediaWidgets
  Qt5::SerialPort
  ${OpenCV_LIBRARIES}
  ${OGRE_LIBRARIES}
)

# 创建主程序可执行文件
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Network
  Qt5::PrintSupport
  Qt5::Multimedia
  Qt5::MultimediaWidgets
  Qt5::SerialPort
  ${OpenCV_LIBRARIES}
  ${OGRE_LIBRARIES}
)

# 安装规则
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "test_*.h" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装资源文件
install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  FILES_MATCHING PATTERN "*.png"
  PATTERN "*.jpg"
)

