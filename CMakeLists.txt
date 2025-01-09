# 基本配置
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
  cv_bridge
  image_transport
)

# 查找Qt5
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)

# 查找OpenCV
find_package(OpenCV 4 REQUIRED)

# 配置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs move_base_msgs actionlib tf2 map_server rviz cv_bridge image_transport
  DEPENDS Qt5Core Qt5Widgets OpenCV
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 启用Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 收集主程序源文件
file(GLOB_RECURSE MAIN_SOURCES
  "src/ros/*.cpp"
  "src/ui/*.cpp"
)

# 收集主程序头文件（排除测试相关的头文件）
file(GLOB_RECURSE HEADERS
  "include/ros/*.h"
  "include/ui/*.h"
)
list(FILTER HEADERS EXCLUDE REGEX ".*test_.*\\.h$")

# 创建主程序库
add_library(${PROJECT_NAME}_lib
  ${MAIN_SOURCES}
  ${HEADERS}
)

# 设置主程序库的Qt自动生成文件目录
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  AUTOMOC_MOC_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}/include"
  AUTOGEN_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/autogen_lib"
  AUTOGEN_SOURCE_GROUP "Generated"
)

# 链接主程序库的依赖
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  ${OpenCV_LIBS}
)

# 创建主程序可执行文件
add_executable(${PROJECT_NAME}
  src/main.cpp
)

# 链接主程序的依赖
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

# 如果启用了测试
if(CATKIN_ENABLE_TESTING)
  # 查找Qt5 Test组件
  find_package(Qt5 REQUIRED COMPONENTS Test)
  
  # 查找GTest
  find_package(GTest REQUIRED)
  
  # 包含测试相关的目录
  include_directories(
    ${Qt5Test_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
  )
  
  # 收集测试源文件
  file(GLOB_RECURSE TEST_SOURCES
    "src/test/*.cpp"
  )
  
  # 收集测试头文件
  file(GLOB_RECURSE TEST_HEADERS
    "include/ui/test_*.h"
  )
  
  # 创建测试可执行文件
  catkin_add_gtest(${PROJECT_NAME}_test
    ${TEST_SOURCES}
    ${TEST_HEADERS}
  )
  
  # 链接测试程序的依赖
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test
      ${PROJECT_NAME}_lib
      ${GTEST_LIBRARIES}
      Qt5::Test
      pthread
    )
  endif()
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lib
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

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

