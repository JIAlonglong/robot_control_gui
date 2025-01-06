# 多机器人通用Qt控制界面

## 项目概述
这是一个基于Qt和ROS开发的多机器人通用控制界面，旨在提供一个灵活、可扩展的图形用户界面，支持多种机器人的远程控制、状态监控、SLAM建图、路径规划等功能。

## 主要功能
- 遥控控制：虚拟摇杆和方向键控制
- 实时状态监控：电量、运行模式等
- SLAM建图和导航
- 传感器数据可视化
- 运动参数曲线显示
- 可扩展的插件系统

## 系统要求
- Ubuntu 20.04 或更高版本
- ROS Noetic
- Qt 5.12 或更高版本
- C++14 或更高版本

## 快速开始
1. 克隆仓库：
```bash
git clone [repository-url]
```

2. 安装依赖：
```bash
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install qt5-default
```

3. 编译：
```bash
cd [workspace]
catkin_make
```

4. 运行：
```bash
source devel/setup.bash
roslaunch robot_control_gui robot_control_gui.launch
```

## 项目结构
```
robot_control_gui/
├── include/          # 头文件
├── src/             # 源代码
│   ├── ui/          # GUI相关代码
│   ├── ros/         # ROS通信相关代码
│   ├── core/        # 核心功能实现
│   └── test/        # 测试代码
├── launch/          # ROS启动文件
├── docs/            # 文档
└── maps/            # 地图文件
```

## 文档
详细文档请参考 `docs` 目录：
- [设计文档](docs/design/README.md)
- [API文档](docs/api/README.md)
- [开发指南](docs/development/README.md)

## 开发规范
- 代码注释遵循Doxygen标准
- 使用Google C++代码风格
- 所有新功能必须包含单元测试
- 提交前必须通过所有测试

## 贡献指南
1. Fork 项目
2. 创建功能分支
3. 提交更改
4. 推送到分支
5. 创建Pull Request

## 许可证
[待定]

## 联系方式
[待补充] 