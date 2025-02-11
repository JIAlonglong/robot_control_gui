# Robot Control GUI

这是一个基于 ROS 和 Qt 的 TurtleBot3 机器人控制界面，提供了直观的可视化控制和状态监控功能。

## ✨ 功能特性

### 🎮 机器人控制
- 虚拟摇杆控制（线速度和角速度）
- 键盘控制（方向键和空格键）
- 紧急停止功能

### 🗺️ 自动定位
- 基于AMCL的自适应定位
- 智能避障系统（20cm安全距离）
- 定位质量评估
- 可视化标记显示

### 🎯 路径规划
- 多种规划算法（Dijkstra、A*、RRT、RRT*）
- 可配置的启发式函数
- 规划参数实时调整
- 规划过程可视化

### 📊 状态监控
- 机器人状态显示（电池、WiFi信号等）
- 速度仪表盘显示
- 传感器数据可视化
- 运行状态监控

## 🔧 系统要求

- Ubuntu 20.04
- ROS Noetic
- Qt 5.12+
- OpenGL 2.1+
- TurtleBot3 相关包

## 🐳 使用Docker

为了简化安装过程,我们提供了预配置的Docker镜像。

### 拉取镜像
```bash
docker pull jialonglong/robot_control_gui:latest
```

### 运行容器
```bash
# 允许Docker访问X服务器
xhost +local:docker

# 运行容器
docker run -it --rm \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /dev:/dev \
    jialonglong/robot_control_gui:latest

# 运行结束后关闭X服务器访问
xhost -local:docker
```

### 开发模式
如果需要在容器中进行开发,可以挂载源代码目录:
```bash
docker run -it --rm \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /dev:/dev \
    -v $(pwd):/root/catkin_ws/src/robot_control_gui \
    jialonglong/robot_control_gui:latest
```

## 📦 安装

1. 克隆仓库：
```bash
cd ~/catkin_ws/src
git clone https://github.com/JIAlonglong/robot_control_gui.git
```

2. 安装依赖：
```bash
sudo apt-get update
sudo apt-get install ros-noetic-rviz ros-noetic-turtlebot3 ros-noetic-turtlebot3-msgs
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🚀 使用方法

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
- 虚拟摇杆：
  - 左侧：控制线速度（上下移动）
  - 右侧：控制角速度（左右移动）
- 键盘控制：
  - ↑：前进
  - ↓：后退
  - ←：左转
  - →：右转
  - 空格：紧急停止

## 📚 文档

- [系统设计](docs/design/README.md)
- [开发指南](docs/development/README.md)
- [API文档](docs/api/README.md)
- [自动定位](docs/auto_localization.md)
- [路径规划](docs/development/path_planning.md)

## 🤝 贡献

欢迎提交问题和改进建议！请查看[开发指南](docs/development/README.md)了解如何参与项目开发。

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👨‍💻 维护者

[@JIAlonglong](https://github.com/JIAlonglong)

## 📝 更新日志

详见 [CHANGELOG.md](CHANGELOG.md) 