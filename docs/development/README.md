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

# 开发指南

## 1. 开发环境搭建

### 1.1 系统要求
- Ubuntu 20.04
- ROS Noetic
- Qt 5.12+
- CMake 3.0.2+
- Python 3.8+

### 1.2 依赖安装
```bash
# 安装ROS基础包
sudo apt-get install ros-noetic-desktop-full

# 安装导航相关包
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-map-server

# 安装Qt开发环境
sudo apt-get install qt5-default
sudo apt-get install qtcreator

# 安装其他依赖
sudo apt-get install python3-catkin-tools
sudo apt-get install python3-vcstool
```

### 1.3 工作空间配置
```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆代码
git clone https://github.com/yourusername/robot_control_gui.git

# 初始化工作空间
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译
catkin build
```

## 2. 开发规范

### 2.1 代码风格
- 遵循 Google C++ 代码规范
- 使用 clang-format 进行代码格式化
- 代码缩进：4个空格
- 行宽限制：100字符

### 2.2 命名规范
```cpp
// 类名：大驼峰
class RobotController {
    // 成员变量：下划线后缀
    int robot_id_;
    double max_velocity_;
    
    // 方法名：小驼峰
    void initializeRobot();
    void setMaxVelocity(double velocity);
};

// 常量：全大写下划线
const int MAX_ROBOT_COUNT = 10;
const double DEFAULT_SPEED = 0.5;
```

### 2.3 注释规范
```cpp
/**
 * @brief 类的简要说明
 * 
 * 类的详细说明，包括用途、功能等
 */
class ExampleClass {
public:
    /**
     * @brief 方法的简要说明
     * @param param1 参数1的说明
     * @param param2 参数2的说明
     * @return 返回值说明
     * @throw std::runtime_error 异常说明
     */
    int exampleMethod(int param1, double param2);
};
```

## 3. 开发流程

### 3.1 功能开发
1. 创建功能分支
```bash
git checkout -b feature/new-feature
```

2. 实现新功能
```cpp
// 添加新的类
class NewFeature {
public:
    void initialize();
    void process();
    
private:
    // 实现细节
};
```

3. 添加测试
```cpp
TEST(NewFeatureTest, InitializationTest) {
    NewFeature feature;
    EXPECT_NO_THROW(feature.initialize());
}
```

4. 提交代码
```bash
git add .
git commit -m "feat: add new feature"
git push origin feature/new-feature
```

### 3.2 代码审查
1. 创建Pull Request
2. 代码审查检查项：
   - 功能完整性
   - 代码质量
   - 测试覆盖率
   - 文档完整性

### 3.3 持续集成
```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      - name: Build
        run: |
          catkin build
          catkin run_tests
```

## 4. 调试指南

### 4.1 日志系统
```cpp
// 使用ROS日志
ROS_DEBUG("Debug message");
ROS_INFO("Info message");
ROS_WARN("Warning message");
ROS_ERROR("Error message");

// 使用Qt日志
qDebug() << "Debug message";
qInfo() << "Info message";
qWarning() << "Warning message";
```

### 4.2 调试工具
```bash
# ROS调试工具
rqt_console              # 日志查看
rqt_graph                # 节点关系图
rqt_plot                 # 数据绘图

# Qt调试工具
qtcreator                # IDE调试
Qt Creator Debugger      # 断点调试
Qt Visual Profiler       # 性能分析
```

### 4.3 常见问题解决
1. 编译错误
```bash
# 清理构建
catkin clean
catkin build --force-cmake

# 检查依赖
rosdep check --from-paths src
```

2. 运行错误
```bash
# 检查ROS环境
printenv | grep ROS

# 检查节点状态
rosnode list
rosnode info /node_name
```

## 5. 发布流程

### 5.1 版本管理
```bash
# 创建版本标签
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0

# 更新版本号
package.xml:
<version>1.0.0</version>
```

### 5.2 发布检查清单
- [ ] 所有测试通过
- [ ] 文档已更新
- [ ] 更新日志已完善
- [ ] 依赖列表已更新
- [ ] 性能测试已完成

### 5.3 部署步骤
```bash
# 1. 构建发布包
catkin_make install

# 2. 创建Docker镜像
docker build -t robot_control_gui:v1.0.0 .

# 3. 推送镜像
docker push yourusername/robot_control_gui:v1.0.0
```

## 6. 性能优化

### 6.1 代码优化
```cpp
// 使用引用避免拷贝
void processData(const std::vector<double>& data);

// 使用移动语义
std::vector<double> getData() && {
    return std::move(data_);
}

// 使用并发处理
std::async(std::launch::async, &Class::heavyTask, this);
```

### 6.2 内存优化
```cpp
// 使用智能指针
std::unique_ptr<RobotController> controller_;
std::shared_ptr<Sensor> sensor_;

// 预分配内存
std::vector<double> data;
data.reserve(expected_size);
```

### 6.3 性能监控
```cpp
// 时间性能
auto start = std::chrono::high_resolution_clock::now();
// ... 执行代码 ...
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

// 内存使用
std::size_t current = getCurrentRSS();
std::size_t peak = getPeakRSS();
``` 