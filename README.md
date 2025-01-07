# Robot Control GUI

这是一个基于 ROS 和 Qt 的 TurtleBot3 机器人控制界面，提供了直观的可视化控制和状态监控功能。

## 功能特性

- 机器人控制
  - 虚拟摇杆控制（线速度和角速度）
  - 键盘控制（方向键和空格键）
  - 紧急停止功能

- 路径规划
  - 多种规划算法（Dijkstra、A*、RRT、RRT*）
  - 可配置的启发式函数
  - 规划参数实时调整
  - 规划过程可视化

- 可视化显示
  - 集成 RViz 显示地图和机器人模型
  - 激光扫描数据可视化
  - 导航路径显示
  - 目标点设置和显示

- 状态监控
  - 机器人状态显示（电池、WiFi信号等）
  - 速度仪表盘
  - 自动检测和提示机器人模型显示问题

## 依赖项

- ROS Noetic
- Qt 5
- RViz
- TurtleBot3 相关包

## 安装

1. 克隆仓库到工作空间：
```bash
cd ~/catkin_ws/src
git clone <repository_url> robot_control_gui
```

2. 安装依赖：
```bash
sudo apt-get install ros-noetic-rviz
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-turtlebot3-msgs
```

3. 编译：
```bash
cd ~/catkin_ws
catkin_make
```

## 使用方法

1. 启动 TurtleBot3 仿真（或实体机器人）：
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

2. 启动控制界面：
```bash
roslaunch robot_control_gui robot_control_gui.launch
```

### 控制说明

- 左侧虚拟摇杆：控制线速度（上下移动）
- 右侧虚拟摇杆：控制角速度（左右移动）
- 键盘控制：
  - ↑：前进
  - ↓：后退
  - ←：左转
  - →：右转
  - 空格：紧急停止

### 显示选项

- 网格显示
- 地图显示
- 机器人模型
- 激光扫描数据
- 导航路径
- 目标点

## 文档

- [开发文档](docs/development/README.md) - 了解详细的开发信息
- [路径规划说明](docs/development/path_planning.md) - 路径规划功能的使用和开发指南
- [更新日志](CHANGELOG.md) - 查看版本更新历史

## 版本

当前版本：0.2.0 - 查看 [更新日志](CHANGELOG.md) 了解详细更改。 