# 路径规划技术文档

## 1. 概述

本文档描述了机器人控制系统中的路径规划模块，包括全局路径规划和局部路径规划的实现细节。

## 2. 全局路径规划

### 2.1 Dijkstra算法

用于在全局地图中找到从起点到目标点的最短路径。

#### 2.1.1 算法描述

```cpp
function dijkstra(graph, start, goal):
    dist[start] = 0
    for each vertex v in graph:
        if v ≠ start: dist[v] = ∞
        prev[v] = undefined
        Q.add_with_priority(v, dist[v])
    
    while Q is not empty:
        u = Q.extract_min()
        if u = goal: break
        for each neighbor v of u:
            alt = dist[u] + length(u, v)
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                Q.decrease_priority(v, alt)
    return dist, prev
```

### 2.2 A*算法

结合启发式信息的改进版Dijkstra算法。

#### 2.2.1 启发函数

```cpp
h(n) = w1 * euclidean_distance(n, goal) + 
       w2 * manhattan_distance(n, goal) +
       w3 * obstacle_cost(n)
```

#### 2.2.2 代价函数

```cpp
f(n) = g(n) + h(n)
```

其中：
- g(n)：从起点到节点n的实际代价
- h(n)：从节点n到目标点的估计代价

## 3. 局部路径规划

### 3.1 DWA算法

动态窗口法（Dynamic Window Approach）用于实时避障和路径优化。

#### 3.1.1 速度采样空间

```cpp
Vs = { (v,w) | v ∈ [v_min, v_max], w ∈ [w_min, w_max] }
```

#### 3.1.2 评价函数

```cpp
G(v,w) = α * heading(v,w) + 
         β * dist(v,w) + 
         γ * velocity(v,w)
```

其中：
- heading：航向得分
- dist：障碍物距离得分
- velocity：速度得分
- α,β,γ：权重系数

### 3.2 TEB算法

时间弹性带（Timed Elastic Band）算法用于轨迹优化。

#### 3.2.1 优化目标

```
minimize J = Σ (w_k * c_k(x))
```

其中：
- c_k：约束函数
- w_k：权重
- x：轨迹参数

#### 3.2.2 约束条件

1. 运动学约束：
   ```
   v ≤ v_max
   |w| ≤ w_max
   |a| ≤ a_max
   ```

2. 动力学约束：
   ```
   |dv/dt| ≤ a_max
   |dw/dt| ≤ ε_max
   ```

## 4. 参数配置

### 4.1 全局规划参数

```yaml
global_planner:
  # Dijkstra参数
  dijkstra:
    allow_unknown: false
    default_tolerance: 0.0
    visualize_potential: false
    use_quadratic: true
    use_grid_path: false
    
  # A*参数
  astar:
    allow_unknown: false
    heuristic_factor: 0.5
    weight_factor: 2.0
    search_factor: 2.0
```

### 4.2 局部规划参数

```yaml
local_planner:
  # DWA参数
  dwa:
    max_vel_x: 0.5
    min_vel_x: 0.0
    max_vel_theta: 1.0
    min_vel_theta: -1.0
    acc_lim_x: 2.5
    acc_lim_theta: 3.2
    
  # TEB参数
  teb:
    max_vel_x: 0.4
    max_vel_x_backwards: 0.2
    max_vel_theta: 0.3
    acc_lim_x: 0.5
    acc_lim_theta: 0.5
```

## 5. 性能优化

### 5.1 计算优化

1. 使用多线程并行计算
2. 采用查找表加速距离计算
3. 实现增量式更新机制

### 5.2 内存优化

1. 使用智能指针管理内存
2. 实现节点池复用机制
3. 采用稀疏图表示

## 6. 调试方法

### 6.1 可视化工具

1. 路径可视化：
   ```cpp
   void visualizePath(const std::vector<Point>& path) {
       nav_msgs::Path msg;
       msg.header.frame_id = "map";
       msg.header.stamp = ros::Time::now();
       
       for(const auto& p : path) {
           geometry_msgs::PoseStamped pose;
           pose.pose.position.x = p.x;
           pose.pose.position.y = p.y;
           msg.poses.push_back(pose);
       }
       path_pub_.publish(msg);
   }
   ```

2. 代价地图可视化：
   ```cpp
   void visualizeCostmap(const Costmap2D& costmap) {
       nav_msgs::OccupancyGrid msg;
       msg.header.frame_id = "map";
       msg.info.resolution = costmap.getResolution();
       msg.info.width = costmap.getSizeInCellsX();
       msg.info.height = costmap.getSizeInCellsY();
       
       for(unsigned int i = 0; i < costmap.getSizeInCellsX() * costmap.getSizeInCellsY(); i++) {
           msg.data.push_back(costmap.getCost(i));
       }
       costmap_pub_.publish(msg);
   }
   ```

### 6.2 性能分析

1. 时间统计：
   ```cpp
   void benchmarkPlanner() {
       auto start = std::chrono::high_resolution_clock::now();
       planner.computePath(start, goal);
       auto end = std::chrono::high_resolution_clock::now();
       
       std::chrono::duration<double> diff = end - start;
       ROS_INFO("Path planning took %f seconds", diff.count());
   }
   ```

2. 内存监控：
   ```cpp
   void memoryUsage() {
       struct rusage usage;
       getrusage(RUSAGE_SELF, &usage);
       ROS_INFO("Memory usage: %ld KB", usage.ru_maxrss);
   }
   ```

## 7. 常见问题

### 7.1 路径规划失败

1. 检查地图数据完整性
2. 验证起点和终点的有效性
3. 调整规划参数
4. 检查障碍物膨胀半径

### 7.2 路径不平滑

1. 增加平滑处理：
   ```cpp
   void smoothPath(std::vector<Point>& path) {
       for(int i = 1; i < path.size()-1; i++) {
           path[i].x = 0.25*path[i-1].x + 0.5*path[i].x + 0.25*path[i+1].x;
           path[i].y = 0.25*path[i-1].y + 0.5*path[i].y + 0.25*path[i+1].y;
       }
   }
   ```

2. 调整TEB参数：
   ```yaml
   teb_local_planner:
     optimization_activate: true
     optimization_verbose: false
     weight_kinematics_forward_drive: 1.0
     weight_kinematics_turning_radius: 1.0
     weight_optimaltime: 1.0
     weight_obstacle: 50.0
   ```

## 8. 参考文献

1. LaValle, S. M. "Planning Algorithms"
2. Thrun, S. et al. "Probabilistic Robotics"
3. Fox, D. et al. "The Dynamic Window Approach to Collision Avoidance" 