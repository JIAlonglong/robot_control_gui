# 开发文档

## 项目结构

```
robot_control_gui/
├── include/          # 头文件
│   ├── ui/          # 界面相关头文件
│   └── ros/         # ROS通信相关头文件
├── src/             # 源代码
│   ├── ui/          # 界面实现
│   ├── ros/         # ROS通信实现
│   ├── test/        # 测试代码
│   └── launch/      # 启动文件
├── config/          # 配置文件
├── rviz/            # RViz配置
└── docs/            # 文档
```

## 主要组件

### 1. 界面组件

#### RVizView
- 文件：`include/ui/rviz_view.h` 和 `src/ui/rviz_view.cpp`
- 功能：集成 RViz 显示，提供地图、机器人模型、激光扫描等可视化
- 特性：
  - 自动检测机器人模型显示问题
  - 支持显示选项控制
  - 支持设置导航目标点

#### JoystickWidget
- 文件：`include/ui/joystick_widget.h` 和 `src/ui/joystick_widget.cpp`
- 功能：虚拟摇杆控制
- 特性：
  - 支持鼠标和触摸操作
  - 可配置活动区域和限制范围
  - 发送位置变化信号

#### RobotStatusPanel
- 文件：`include/ui/robot_status_panel.h` 和 `src/ui/robot_status_panel.cpp`
- 功能：显示机器人状态信息
- 特性：
  - 电池电量显示
  - WiFi信号强度显示
  - 运行状态显示

#### SpeedDashboard
- 文件：`include/ui/speed_dashboard.h` 和 `src/ui/speed_dashboard.cpp`
- 功能：速度仪表盘显示
- 特性：
  - 线速度显示
  - 角速度显示

### 2. ROS通信

#### RobotController
- 文件：`include/ros/robot_controller.h` 和 `src/ros/robot_controller.cpp`
- 功能：处理与ROS系统的通信
- 特性：
  - 发布速度命令
  - 处理导航目标
  - 管理机器人状态
  - 处理传感器数据

## 开发指南

### 添加新的显示组件

1. 在 `include/ui` 目录下创建头文件
2. 在 `src/ui` 目录下创建实现文件
3. 在 `MainWindow` 类中集成新组件
4. 在 `CMakeLists.txt` 中添加新文件

### 添加新的ROS功能

1. 在 `RobotController` 类中添加新的方法
2. 添加必要的ROS消息类型
3. 实现消息处理函数
4. 在界面中添加相应的控制元素

### 编码规范

- 使用驼峰命名法
- 类名首字母大写
- 成员变量以下划线结尾
- 添加必要的注释
- 保持代码整洁和可读性

### 测试

- 单元测试位于 `src/test` 目录
- 使用 Google Test 框架
- 运行测试：`catkin build --catkin-make-args run_tests`

## 调试提示

### 常见问题

1. 机器人模型不可见
   - 检查 robot_description 参数
   - 检查 TF 变换
   - 检查固定坐标系设置

2. 地图显示问题
   - 确认地图服务器是否运行
   - 检查地图话题是否正确
   - 验证坐标系转换

3. 控制问题
   - 检查 cmd_vel 话题
   - 验证速度限制设置
   - 确认机器人状态 