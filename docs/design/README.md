# 机器人控制系统设计文档

## 1. 系统架构

### 1.1 整体架构

系统采用分层架构设计，主要包含以下层次：

1. 用户界面层（GUI）
   - 机器人状态显示
   - 控制面板
   - 地图显示
   - 参数配置界面

2. 业务逻辑层
   - 机器人控制器（RobotController）
   - 导航管理器（NavigationManager）
   - 定位管理器（LocalizationManager）
   - 地图管理器（MapManager）

3. ROS通信层
   - 话题发布/订阅
   - 服务调用
   - Action客户端

4. 硬件抽象层
   - 激光雷达驱动
   - 电机控制
   - 传感器接口

### 1.2 核心模块

#### 1.2.1 RobotController

负责机器人的核心控制逻辑：
- 运动控制
- 状态监控
- 安全管理
- 任务调度

#### 1.2.2 NavigationPanel

提供导航相关的用户界面：
- 目标点设置
- 路径显示
- 导航参数配置
- 状态反馈

#### 1.2.3 LocalizationManager

管理机器人定位相关功能：
- AMCL参数配置
- 自动定位控制
- 位姿估计
- 定位质量评估

## 2. 模块设计

### 2.1 自动定位模块

#### 2.1.1 类图
```
+----------------+     +------------------+     +----------------+
| RobotController|     |LocalizationPanel|     |  AmclWrapper  |
+----------------+     +------------------+     +----------------+
| -is_localizing |     | -status_label   |     | -min_particles|
| -safety_dist   |     | -progress_bar   |     | -max_particles|
+----------------+     +------------------+     +----------------+
| +startAutoLoc()|     | +updateStatus() |     | +setParams()  |
| +stopAutoLoc() |     | +showProgress() |     | +getStatus()  |
+----------------+     +------------------+     +----------------+
```

#### 2.1.2 状态流转
```
初始状态 -> 全局定位 -> 运动探索 -> 位姿优化 -> 定位完成
   ↑          |            |           |          |
   +----------+            |           |          |
              +------------+           |          |
              +------------------------+          |
              +----------------------------------|
```

### 2.2 避障模块

#### 2.2.1 类图
```
+----------------+     +------------------+     +----------------+
| ObstacleAvoider|     |   LaserScanner  |     | MotionPlanner |
+----------------+     +------------------+     +----------------+
| -safety_dist   |     | -scan_data      |     | -linear_vel   |
| -is_avoiding   |     | -range_max      |     | -angular_vel  |
+----------------+     +------------------+     +----------------+
| +checkSafety() |     | +processScan()  |     | +planMotion() |
| +avoid()       |     | +getRanges()    |     | +execute()    |
+----------------+     +------------------+     +----------------+
```

## 3. 关键流程

### 3.1 自动定位流程

1. 初始化
   - 配置AMCL参数
   - 清除代价地图
   - 初始化可视化标记

2. 全局定位
   - 触发全局定位
   - 分散粒子
   - 等待粒子收敛

3. 运动探索
   - 在安全范围内运动
   - 收集传感器数据
   - 更新位姿估计

4. 定位完成
   - 验证定位质量
   - 设置初始位姿
   - 发布定位结果

### 3.2 避障流程

1. 数据处理
   - 接收激光数据
   - 分区域处理
   - 计算安全距离

2. 决策控制
   - 判断是否需要避障
   - 选择避障方向
   - 计算运动指令

3. 执行控制
   - 发布速度指令
   - 监控执行状态
   - 调整运动参数

## 4. 配置管理

### 4.1 关键参数

```yaml
# 定位参数
localization:
  min_particles: 10000
  max_particles: 20000
  update_min_d: 0.05
  update_min_a: 0.1
  recovery_alpha: 0.1

# 安全参数
safety:
  distance: 0.2
  max_linear_vel: 0.3
  max_angular_vel: 0.5
  motion_radius: 0.15

# 运动控制参数
motion:
  k1: 0.3
  k2: 0.8
  k3: 0.2
```

### 4.2 配置文件结构

```
config/
  ├── amcl/
  │   ├── amcl_params.yaml
  │   └── particle_filter.yaml
  ├── navigation/
  │   ├── costmap_common.yaml
  │   ├── local_costmap.yaml
  │   └── global_costmap.yaml
  └── robot/
      ├── safety_params.yaml
      └── motion_params.yaml
```

## 5. 异常处理

### 5.1 定位异常

1. 粒子退化
   - 增加粒子数量
   - 调整重采样阈值
   - 重新触发全局定位

2. 传感器异常
   - 检查数据有效性
   - 降级处理
   - 故障恢复

### 5.2 避障异常

1. 传感器故障
   - 紧急停止
   - 切换备用传感器
   - 报警提示

2. 控制异常
   - 限制速度
   - 重置控制器
   - 切换控制模式

## 6. 测试方案

### 6.1 单元测试

1. 模块测试
   - 定位算法测试
   - 避障逻辑测试
   - 运动控制测试

2. 接口测试
   - ROS通信测试
   - 参数配置测试
   - 异常处理测试

### 6.2 集成测试

1. 功能测试
   - 自动定位测试
   - 避障功能测试
   - 导航功能测试

2. 性能测试
   - 定位精度测试
   - 实时性测试
   - 稳定性测试

## 7. 部署说明

### 7.1 环境要求

- ROS Noetic
- Ubuntu 20.04
- Qt 5.12+
- C++14及以上

### 7.2 依赖安装

```bash
# 安装ROS依赖
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-map-server

# 安装Qt依赖
sudo apt-get install qt5-default
sudo apt-get install libqt5widgets5
```

### 7.3 编译部署

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 运行
source devel/setup.bash
roslaunch robot_control_gui robot_control.launch
``` 