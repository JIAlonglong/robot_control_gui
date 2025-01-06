cmake_minimum_required(VERSION 3.10)
project(robot_control_gui)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Qt包
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# 启用Qt的MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 添加ROS依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
)

# 声明catkin包
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs
)

# 添加包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

# 头文件
set(HEADERS
    include/ui/joystick_widget.h
    include/ui/test_window.h
    include/ui/robot_status_panel.h
    include/ui/test_robot_status_panel.h
    include/ui/speed_dashboard.h
    include/ui/test_dashboard_window.h
    include/ui/main_window.h
    include/ros/robot_controller.h
    include/ui/map_view.h
)

# 源文件
set(SOURCES
    src/ui/joystick_widget.cpp
    src/ui/test_window.cpp
    src/ui/robot_status_panel.cpp
    src/ui/test_robot_status_panel.cpp
    src/ui/speed_dashboard.cpp
    src/ui/test_dashboard_window.cpp
    src/ui/main_window.cpp
    src/ros/robot_controller.cpp
    src/ui/map_view.cpp
)

# 测试程序源文件
set(TEST_SOURCES
    src/test/test_joystick.cpp
)

# 添加测试程序源文件
set(TEST_ROBOT_STATUS_PANEL_SOURCES
    src/test/test_robot_status_panel_main.cpp
)

# 添加测试程序源文件
set(TEST_DASHBOARD_SOURCES
    src/test/test_dashboard_main.cpp
)

# 添加地图显示测试程序源文件
set(TEST_MAP_VIEW_SOURCES
    src/test/test_map_view.cpp
)

# 主库
add_library(${PROJECT_NAME} STATIC 
    ${HEADERS}
    ${SOURCES}
)

# 确保MOC文件被正确生成
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# 测试程序
add_executable(test_joystick ${TEST_SOURCES})
target_link_libraries(test_joystick PRIVATE
    ${PROJECT_NAME}
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# 测试程序
add_executable(test_robot_status_panel ${TEST_ROBOT_STATUS_PANEL_SOURCES})
target_link_libraries(test_robot_status_panel PRIVATE
    ${PROJECT_NAME}
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# 测试程序
add_executable(test_dashboard ${TEST_DASHBOARD_SOURCES})
target_link_libraries(test_dashboard PRIVATE
    ${PROJECT_NAME}
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# 地图显示测试程序
add_executable(test_map_view ${TEST_MAP_VIEW_SOURCES})
target_link_libraries(test_map_view PRIVATE
    ${PROJECT_NAME}
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# 主程序
add_executable(robot_control_gui_app src/main.cpp)
target_link_libraries(robot_control_gui_app PRIVATE
    ${PROJECT_NAME}
    Qt5::Widgets
    ${catkin_LIBRARIES}
)
