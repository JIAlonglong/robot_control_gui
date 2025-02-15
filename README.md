# Robot Control GUI

这是一个基于 ROS 和 Qt 的 TurtleBot3 机器人控制界面，提供了直观的可视化控制和状态监控功能。

## ✨ 功能特性

### 🌐 网络配置
- 支持主从机模式配置
- 自动检测本地网络接口
- 可视化的连接状态监控
- 支持多机器人协同

### 🎮 机器人控制
- 虚拟摇杆控制（线速度和角速度）
- 键盘控制（方向键和空格键）
- 紧急停止功能
- 碰撞避免系统

### 🗺️ 多种建图方式
- Gmapping SLAM
- Cartographer SLAM
- Hector SLAM
- 手动遥控建图
- 实时地图预览和保存

### 🎯 智能导航系统
- 基于AMCL的自适应定位
- 多种规划算法（Dijkstra、A*、RRT、RRT*）
- 智能避障系统（20cm安全距离）
- 定位质量评估
- 可视化标记显示

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

### 网络配置
1. 主机模式（控制电脑）：
```bash
# 选择"主机模式 (Master)"
# 设置本机IP（自动检测可用网络接口）
# ROS_MASTER_URI 将被自动配置
```

2. 从机模式（机器人）：
```bash
# 选择"从机模式 (Slave)"
# 设置主控电脑IP
# 配置本机IP
# 确保与主机在同一网段
```

### 启动步骤
1. 启动 TurtleBot3 仿真（或实体机器人）：
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

2. 启动控制界面：
```bash
roslaunch robot_control_gui robot_control_gui.launch
```

### 建图操作
1. 选择建图方法（Gmapping/Cartographer/Hector）
2. 配置建图参数
3. 使用虚拟摇杆或键盘控制机器人建图
4. 实时查看建图效果
5. 完成后保存地图

### 导航控制
1. 加载已有地图
2. 设置初始位置
3. 选择导航算法
4. 点击目标位置开始导航
5. 监控导航状态

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
- [网络配置](docs/network_configuration.md)
- [自动定位](docs/auto_localization.md)
- [路径规划](docs/development/path_planning.md)
- [建图指南](docs/mapping_guide.md)

## 🤝 贡献

欢迎提交问题和改进建议！请查看[开发指南](docs/development/README.md)了解如何参与项目开发。

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👨‍💻 维护者

[@JIAlonglong](https://github.com/JIAlonglong)

## 📝 更新日志

详见 [CHANGELOG.md](CHANGELOG.md) 