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
- C++14及以上

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
```

### 1.3 工作空间配置

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆代码
git clone https://github.com/your-repo/robot_control_gui.git

# 编译
cd ~/catkin_ws
catkin_make
```

## 2. 代码结构

```
robot_control_gui/
├── include/
│   ├── gui/
│   │   ├── main_window.h
│   │   ├── navigation_panel.h
│   │   └── visualization_panel.h
│   └── ros/
│       ├── robot_controller.h
│       └── ros_bridge.h
├── src/
│   ├── gui/
│   │   ├── main_window.cpp
│   │   ├── navigation_panel.cpp
│   │   └── visualization_panel.cpp
│   └── ros/
│       ├── robot_controller.cpp
│       └── ros_bridge.cpp
├── launch/
│   └── robot_control.launch
├── config/
│   ├── rviz/
│   └── params/
└── CMakeLists.txt
```

## 3. 开发规范

### 3.1 代码风格

- 使用Google C++代码风格
- 类名使用大驼峰命名法
- 函数名使用小驼峰命名法
- 变量名使用下划线命名法
- 常量使用全大写加下划线

### 3.2 注释规范

```cpp
/**
 * @brief 函数简要说明
 * @param param1 参数1说明
 * @param param2 参数2说明
 * @return 返回值说明
 */
```

### 3.3 Git提交规范

```
feat: 添加新功能
fix: 修复bug
docs: 更新文档
style: 代码格式修改
refactor: 代码重构
test: 添加测试
chore: 构建过程或辅助工具的变动
```

## 4. 功能开发流程

### 4.1 GUI开发

1. 创建新的面板类：
```cpp
class NewPanel : public QWidget {
    Q_OBJECT
public:
    explicit NewPanel(QWidget* parent = nullptr);
    
signals:
    void signalName();
    
private slots:
    void onButtonClicked();
    
private:
    void setupUi();
};
```

2. 在主窗口中添加面板：
```cpp
void MainWindow::setupUi() {
    auto* new_panel = new NewPanel(this);
    ui_->tab_widget->addTab(new_panel, tr("新面板"));
}
```

### 4.2 ROS功能开发

1. 创建新的功能类：
```cpp
class NewFeature {
public:
    NewFeature(ros::NodeHandle& nh);
    
private:
    void callback(const msg_type::ConstPtr& msg);
    ros::Subscriber sub_;
    ros::Publisher pub_;
};
```

2. 在RobotController中集成：
```cpp
void RobotController::setupNewFeature() {
    new_feature_ = std::make_unique<NewFeature>(nh_);
}
```

## 5. 测试指南

### 5.1 单元测试

使用Google Test框架：

```cpp
TEST(TestSuite, TestName) {
    // 准备测试数据
    auto obj = TestClass();
    
    // 执行测试
    auto result = obj.testMethod();
    
    // 验证结果
    EXPECT_EQ(result, expected_value);
}
```

### 5.2 集成测试

使用rostest框架：

```xml
<launch>
  <test test-name="test_name" pkg="robot_control_gui" type="test_node" />
</launch>
```

## 6. 调试技巧

### 6.1 ROS调试

1. 使用rqt工具：
```bash
rqt_console  # 查看日志
rqt_graph    # 查看节点关系
rqt_plot     # 绘制数据曲线
```

2. 使用rostopic工具：
```bash
rostopic echo /topic_name  # 查看话题数据
rostopic hz /topic_name   # 查看话题频率
```

### 6.2 Qt调试

1. 使用QDebug：
```cpp
qDebug() << "Debug message";
qWarning() << "Warning message";
qCritical() << "Critical message";
```

2. 使用Qt Creator调试器：
- 设置断点
- 查看变量
- 单步执行

## 7. 发布流程

### 7.1 版本管理

```bash
# 创建新版本
git tag -a v1.0.0 -m "版本说明"
git push origin v1.0.0

# 更新版本号
package.xml
CMakeLists.txt
```

### 7.2 发布检查清单

1. 代码审查
   - 代码风格检查
   - 单元测试通过
   - 集成测试通过

2. 文档更新
   - API文档
   - 使用说明
   - 更新日志

3. 依赖检查
   - 检查依赖版本
   - 更新package.xml

4. 性能测试
   - CPU使用率
   - 内存占用
   - 响应时间 