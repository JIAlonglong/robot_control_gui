/**
 * @file test_map_view.cpp
 * @brief 测试地图显示功能
 */

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QTimer>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "ui/map_view.h"

class MapViewTest {
public:
    MapViewTest(MapView* view) : map_view_(view) {}

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_view_->updateMap(msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        map_view_->updateRobotPose(msg);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        map_view_->updateLaserScan(msg);
    }

private:
    MapView* map_view_;
};

int main(int argc, char *argv[]) {
    // 初始化ROS节点
    ros::init(argc, argv, "test_map_view");
    ros::NodeHandle nh;

    // 初始化Qt应用
    QApplication app(argc, argv);
    
    // 创建主窗口
    QMainWindow window;
    window.setWindowTitle("Map View Test");
    
    // 创建中心部件
    QWidget* central = new QWidget(&window);
    window.setCentralWidget(central);
    
    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(central);
    
    // 创建地图显示组件
    MapView* map_view = new MapView(central);
    layout->addWidget(map_view);
    
    // 设置窗口大小
    window.resize(800, 600);
    window.show();

    // 创建测试类实例
    MapViewTest test(map_view);

    // 创建ROS定时器，用于处理ROS消息
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, []() {
        ros::spinOnce();
    });
    ros_timer.start(100);  // 10Hz

    // 订阅相关话题
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
        "map", 1, &MapViewTest::mapCallback, &test);
    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "odom", 10, &MapViewTest::odomCallback, &test);
    
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(
        "scan", 10, &MapViewTest::scanCallback, &test);

    return app.exec();
} 