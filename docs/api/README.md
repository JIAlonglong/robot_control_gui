# API文档

## ROS接口

### 1. 发布的话题（Publishers）

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/cmd_vel` | geometry_msgs/Twist | 机器人速度控制命令 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 机器人初始位姿 |
| `/localization_markers` | visualization_msgs/MarkerArray | 定位可视化标记 |

### 2. 订阅的话题（Subscribers）

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/scan` | sensor_msgs/LaserScan | 激光雷达数据 |
| `/odom` | nav_msgs/Odometry | 里程计数据 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL位姿估计 |
| `/map` | nav_msgs/OccupancyGrid | 地图数据 |

### 3. 服务（Services）

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/global_localization` | std_srvs/Empty | 触发全局定位 |
| `/clear_costmaps` | std_srvs/Empty | 清除代价地图 |

### 4. Action接口

| Action名称 | Action类型 | 描述 |
|-----------|-----------|------|
| `/move_base` | move_base_msgs/MoveBaseAction | 导航控制 |

## Qt界面API

### 1. 信号（Signals）

```cpp
// 位姿更新信号
void poseUpdated(const geometry_msgs::Pose& pose);

// 定位状态信号
void localizationStatusChanged(const QString& status);
void localizationProgressChanged(double progress);

// 导航状态信号
void navigationStatusChanged(const QString& status);
void navigationProgressChanged(double progress);
```

### 2. 槽函数（Slots）

```cpp
// 控制命令槽
void setLinearVelocity(double linear);
void setAngularVelocity(double angular);
void stop();

// 定位控制槽
void startAutoLocalization();
void cancelAutoLocalization();

// 导航控制槽
void setNavigationGoal(const geometry_msgs::PoseStamped& goal);
void cancelNavigation();
```

### 3. 属性（Properties）

```cpp
// 机器人状态
bool isInitialized() const;
bool isNavigating() const;
bool isLocalized() const;

// 运动参数
double getLinearVelocity() const;
double getAngularVelocity() const;
double getMaxLinearVelocity() const;
double getMaxAngularVelocity() const;
```

## 参数配置API

### 1. 动态参数

使用dynamic_reconfigure配置以下参数：

```yaml
# AMCL参数
min_particles: 10000
max_particles: 20000
update_min_d: 0.05
update_min_a: 0.1

# 安全参数
safety_distance: 0.2
max_linear_vel: 0.3
max_angular_vel: 0.5
```

### 2. 静态参数

在launch文件中配置：

```xml
<param name="base_frame" value="base_footprint"/>
<param name="odom_frame" value="odom"/>
<param name="map_frame" value="map"/>
<param name="use_sim_time" value="false"/>
``` 