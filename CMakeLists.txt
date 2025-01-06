cmake_minimum_required(VERSION 3.0.2)
project(robot_control_gui)

# 使用C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  map_server
  move_base_msgs
  actionlib
  std_srvs
  turtlebot3_bringup
  turtlebot3_navigation
  turtlebot3_slam
  turtlebot3_teleop
  image_transport
  cv_bridge
  rostest
)

# 查找Qt5
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  Network
  Test
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
    sensor_msgs
    tf2
    tf2_ros
    map_server
    move_base_msgs
    actionlib
    std_srvs
    turtlebot3_bringup
    turtlebot3_navigation
    turtlebot3_slam
    turtlebot3_teleop
    image_transport
    cv_bridge
  DEPENDS Qt5Core Qt5Gui Qt5Widgets Qt5Network
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
)

# 启用Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 收集源文件
set(SOURCES
  src/main.cpp
  src/ui/joystick_widget.cpp
  src/ui/main_window.cpp
  src/ui/navigation_panel.cpp
  src/ui/robot_status_panel.cpp
  src/ui/speed_dashboard.cpp
  src/ui/map_view.cpp
  src/ros/robot_controller.cpp
)

# 收集头文件
set(HEADERS
  include/ui/joystick_widget.h
  include/ui/main_window.h
  include/ui/navigation_panel.h
  include/ui/robot_status_panel.h
  include/ui/speed_dashboard.h
  include/ui/map_view.h
  include/ros/robot_controller.h
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
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_app src/main.cpp)
target_link_libraries(${PROJECT_NAME}_app
  ${PROJECT_NAME}
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

# 添加Python脚本的安装规则
catkin_install_python(PROGRAMS
  scripts/publish_test_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 测试配置
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # 添加gtest依赖
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  # 地图显示测试
  add_rostest_gtest(test_map_view
    src/launch/test/test_map_view.launch
    src/test/test_map_view.cpp
    ${SOURCES}
  )
  target_link_libraries(test_map_view
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Test
  )
endif()

