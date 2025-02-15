# 建图功能使用指南

## 支持的建图方法

### 1. Gmapping SLAM
- 基于粒子滤波的激光SLAM
- 适用场景：室内环境，走廊，办公室
- 特点：
  - 计算量适中
  - 回环检测能力强
  - 地图精度高

### 2. Cartographer SLAM
- Google开发的高精度SLAM系统
- 适用场景：大型室内环境，复杂场景
- 特点：
  - 实时回环检测
  - 多传感器融合
  - 高精度建图
  - 资源占用较大

### 3. Hector SLAM
- 无需里程计的激光SLAM
- 适用场景：小型室内环境，应急场景
- 特点：
  - 计算量小
  - 无需里程计
  - 实时性好
  - 对环境要求较高

## 建图参数配置

### 通用参数
```yaml
# 地图参数
map_resolution: 0.05      # 地图分辨率(m/pixel)
map_update_interval: 5.0  # 地图更新间隔(s)
map_size: 2048           # 地图大小(pixel)

# 激光雷达参数
laser_max_range: 30.0    # 最大测量范围(m)
laser_min_range: 0.1     # 最小测量范围(m)
laser_max_angle: 180     # 最大扫描角度(度)
laser_min_angle: -180    # 最小扫描角度(度)
```

### Gmapping 特有参数
```yaml
particles: 30            # 粒子数量
minimumScore: 200       # 最小匹配分数
linearUpdate: 0.5       # 线性更新阈值(m)
angularUpdate: 0.5      # 角度更新阈值(rad)
```

### Cartographer 特有参数
```yaml
num_range_data: 100     # 范围数据数量
optimization_problem_options:
  huber_scale: 1e1      # Huber损失函数参数
  optimization_interval: 3  # 优化间隔
```

### Hector 特有参数
```yaml
map_resolution: 0.025    # 地图分辨率(m/pixel)
map_size: 1024          # 地图大小(pixel)
map_multi_res_levels: 3  # 多分辨率层级
```

## 建图操作流程

### 1. 准备工作
```bash
# 启动机器人底盘和传感器
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# 启动控制界面
roslaunch robot_control_gui robot_control_gui.launch
```

### 2. 选择建图方法
1. 在建图面板选择合适的SLAM方法
2. 配置相关参数
3. 点击"开始建图"

### 3. 控制机器人建图
- 使用虚拟摇杆控制
  - 左侧摇杆：控制线速度
  - 右侧摇杆：控制角速度
- 使用键盘控制
  - ↑：前进
  - ↓：后退
  - ←：左转
  - →：右转
  - 空格：紧急停止

### 4. 建图技巧
1. 保持适当速度
   - 线速度建议：0.2-0.3 m/s
   - 角速度建议：0.3-0.5 rad/s

2. 环境探索策略
   - 先走外围轮廓
   - 再探索内部区域
   - 注意回环检测

3. 注意事项
   - 避免急转弯
   - 保持适当重叠
   - 注意环境光线
   - 避免动态障碍物

### 5. 保存地图
```bash
# 自动保存
点击"保存地图"按钮

# 手动保存（终端）
rosrun map_server map_saver -f map_name
```

## 建图质量评估

### 1. 视觉评估
- 墙壁直线度
- 角落清晰度
- 空白区域大小
- 噪点情况

### 2. 数值评估
- 地图分辨率
- 占用栅格比例
- 未知区域比例
- 闭环误差

## 常见问题

### 1. 建图失败
- 检查传感器数据
- 验证TF转换
- 确认参数设置

### 2. 地图畸变
- 降低运动速度
- 调整更新参数
- 检查环境光线

### 3. 回环检测失败
- 增加重叠区域
- 调整匹配参数
- 优化运动路径

### 4. 资源占用过高
- 调整粒子数量
- 降低更新频率
- 优化地图大小

## 性能优化

### 1. 计算资源优化
- 调整粒子数量
- 优化更新频率
- 合理设置地图大小

### 2. 精度优化
- 调整传感器参数
- 优化匹配算法参数
- 改进运动控制策略

### 3. 内存优化
- 合理设置地图大小
- 优化数据结构
- 及时清理缓存

## 调试工具

### 1. RViz 可视化
- 查看激光数据
- 监控位姿估计
- 观察地图构建

### 2. rqt 工具
- 查看计算资源
- 监控话题频率
- 分析节点关系

### 3. 日志分析
- 查看警告信息
- 分析错误原因
- 追踪性能问题

## 建图最佳实践

### 环境准备
1. 光线条件
   - 保持稳定的光照
   - 避免强光直射
   - 处理反光表面

2. 空间布置
   - 移除动态障碍物
   - 确保通道畅通
   - 标记特殊区域

3. 传感器设置
   - 校准激光雷达
   - 检查安装位置
   - 验证数据质量

### 建图策略

1. 预规划路线
   ```
   ┌──────────┐
   │          │
   │  ┌────┐  │
   │  │    │  │
   │  └────┘  │
   │          │
   └──────────┘
   建议路线：沿外围→内部→细节
   ```

2. 速度控制
   - 直线行驶：0.2-0.3 m/s
   - 转弯时：0.1-0.2 m/s
   - 角速度：最大0.5 rad/s

3. 回环检测
   - 每隔一段距离回到已建图区域
   - 保持30-50%的区域重叠
   - 避免长时间无回环

### 质量控制

1. 实时监控
   - 观察点云密度
   - 检查回环匹配
   - 监控CPU占用

2. 定期检查
   - 每5-10分钟检查地图质量
   - 验证关键区域精度
   - 及时处理问题

3. 地图优化
   - 使用地图编辑工具
   - 清理噪点
   - 优化占用栅格

## 高级功能

### 1. 多分辨率建图
```yaml
# 配置示例
map_levels:
  - resolution: 0.05  # 精细地图
  - resolution: 0.1   # 导航用图
  - resolution: 0.2   # 全局规划
```

### 2. 自动回环检测
```yaml
loop_closure:
  min_dist: 1.0      # 最小回环距离
  min_score: 0.8     # 最小匹配分数
  search_radius: 3.0 # 搜索半径
```

### 3. 地图合并
```bash
# 合并多个地图
rosrun map_server map_merger map1.yaml map2.yaml -o merged_map
```

## 性能调优

### 1. 资源使用优化
```yaml
# Gmapping 优化示例
particles: 30            # 默认值：100
linearUpdate: 0.5        # 默认值：0.3
angularUpdate: 0.5       # 默认值：0.3
temporalUpdate: 3.0      # 默认值：-1.0
resampleThreshold: 0.5   # 默认值：0.5
```

### 2. 精度优化
```yaml
# 激光雷达优化
scan_filter:
  min_range: 0.1
  max_range: 30.0
  angle_increment: 0.01
  time_increment: 0.0001
```

### 3. 内存优化
```yaml
# 地图优化
map_optimization:
  cache_size: 100
  thread_count: 4
  buffer_size: 1024
```

## 调试技巧

### 1. 常用命令
```bash
# 查看转换树
rosrun tf view_frames

# 检查话题延迟
rostopic hz /scan

# 查看资源占用
htop
```

### 2. 日志分析
```bash
# 查看ROS日志
rqt_console

# 保存调试信息
rosrun rqt_logger_level rqt_logger_level
```

### 3. 性能分析
```bash
# 话题带宽分析
rostopic bw /scan

# 节点性能分析
rosrun rqt_graph rqt_graph
``` 