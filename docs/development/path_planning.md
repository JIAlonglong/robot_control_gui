# 路径规划功能开发指南

## 功能概述

路径规划模块提供了多种路径规划算法的选择和参数配置功能，支持以下特性：

1. 多种规划算法选择：
   - Dijkstra算法：基于图搜索的最短路径算法
   - A*算法：启发式搜索算法，支持多种启发函数
   - RRT算法：快速随机树算法，适用于高维空间
   - RRT*算法：RRT的优化版本，提供渐进最优解

2. 启发式函数选择（用于A*算法）：
   - 欧几里得距离：直线距离
   - 曼哈顿距离：网格距离
   - 对角线距离：考虑对角线移动的距离

3. 规划参数配置：
   - 规划时间限制：设置路径规划的最大允许时间
   - 路径插值距离：设置路径点之间的最小距离
   - 未知区域处理：是否允许规划穿过未知区域
   - 可视化选项：是否显示规划过程

## 使用说明

1. 选择规划算法：
   - 在"规划器"下拉菜单中选择所需的算法
   - 不同算法适用于不同场景：
     * Dijkstra：适用于需要找到最短路径的场景
     * A*：适用于需要快速找到较优路径的场景
     * RRT：适用于复杂环境中的路径规划
     * RRT*：适用于需要优化路径质量的场景

2. 配置启发式函数（仅A*算法）：
   - 欧几里得距离：适用于机器人可以全向移动的场景
   - 曼哈顿距离：适用于机器人只能沿正交方向移动的场景
   - 对角线距离：适用于机器人可以斜向移动的场景

3. 调整规划参数：
   - 规划时间限制：根据实际需求设置，通常5秒足够
   - 路径插值距离：较小的值会生成更平滑的路径，但计算量更大
   - 未知区域：根据实际场景决定是否允许穿过未知区域
   - 可视化：调试时建议开启，生产环境可以关闭以提高性能

## 开发指南

### 添加新的规划算法

1. 在`navigation_panel.h`中添加新的算法选项：
```cpp
planner_type_->addItem(tr("新算法名称"), "new_planner_name");
```

2. 在move_base配置文件中添加新的规划器插件：
```yaml
base_global_planner: new_planner_name/NewPlannerROS
```

3. 实现新的规划器插件：
```cpp
#include <nav_core/base_global_planner.h>

namespace new_planner_name {
class NewPlannerROS : public nav_core::BaseGlobalPlanner {
public:
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal,
                 std::vector<geometry_msgs::PoseStamped>& plan);
};
}
```

### 添加新的启发式函数

1. 在`navigation_panel.h`中添加新的启发式函数选项：
```cpp
heuristic_type_->addItem(tr("新启发式函数"), "new_heuristic");
```

2. 在规划器配置中实现新的启发式函数：
```cpp
double calculateHeuristic(const Node& current, const Node& goal) {
    // 实现新的启发式函数
}
```

### 添加新的参数配置

1. 在UI中添加新的参数控件：
```cpp
QLineEdit* new_param_ = new QLineEdit(default_value);
params_layout->addWidget(new QLabel(tr("新参数:")));
params_layout->addWidget(new_param_);
```

2. 添加参数变更处理：
```cpp
connect(new_param_, &QLineEdit::textChanged,
        this, &NavigationPanel::onNewParamChanged);
```

3. 实现参数更新函数：
```cpp
void NavigationPanel::onNewParamChanged(const QString& text) {
    robot_controller_->setParam("/move_base/new_param", text.toDouble());
}
```

## 调试建议

1. 使用RViz可视化工具：
   - 显示全局规划路径：添加Path显示，话题为 `/move_base/GlobalPlanner/plan`
   - 显示局部规划路径：添加Path显示，话题为 `/move_base/LocalPlanner/local_plan`
   - 显示代价地图：添加Map显示，话题为 `/move_base/global_costmap/costmap`

2. 使用rqt_reconfigure动态调整参数：
   - 运行 `rosrun rqt_reconfigure rqt_reconfigure`
   - 选择 `/move_base` 节点
   - 实时调整参数并观察效果

3. 使用rosbag记录和回放：
   - 记录规划数据：`rosbag record /move_base/*`
   - 回放数据：`rosbag play planning_data.bag`

## 常见问题

1. 规划失败：
   - 检查代价地图更新是否正常
   - 确认起点和目标点是否在可行区域内
   - 调整规划时间限制

2. 路径不平滑：
   - 减小路径插值距离
   - 调整局部规划器参数
   - 考虑使用平滑后处理器

3. 规划过慢：
   - 增大路径插值距离
   - 减小规划区域范围
   - 关闭可视化选项

4. 穿越障碍物：
   - 检查障碍物膨胀参数
   - 调整代价地图更新频率
   - 确认传感器数据是否正常

## 性能优化

1. 参数调优：
   - 根据实际场景调整规划时间限制
   - 选择合适的路径插值距离
   - 在不需要时关闭可视化

2. 算法选择：
   - 简单环境：使用A*算法
   - 复杂环境：使用RRT*算法
   - 高速导航：使用简化的Dijkstra算法

3. 代码优化：
   - 使用多线程处理规划请求
   - 实现缓存机制
   - 优化代价地图更新策略 