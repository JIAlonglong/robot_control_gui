/**
 * @file test_map_view.cpp
 * @brief 测试地图显示功能
 */

#include <gtest/gtest.h>
#include <QApplication>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "ui/main_window.h"

class MapViewTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化ROS节点
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "map_view_test");
        
        // 创建节点句柄
        nh_ = std::make_shared<ros::NodeHandle>();
        
        // 创建发布器
        map_pub_ = nh_->advertise<nav_msgs::OccupancyGrid>("/map", 1);
        odom_pub_ = nh_->advertise<nav_msgs::Odometry>("/odom", 1);
        scan_pub_ = nh_->advertise<sensor_msgs::LaserScan>("/scan", 1);
        
        // 等待发布器准备就绪
        ros::Duration(0.5).sleep();
    }

    void TearDown() override {
        // 关闭发布器
        map_pub_.shutdown();
        odom_pub_.shutdown();
        scan_pub_.shutdown();
        
        // 关闭ROS节点
        ros::shutdown();
    }

    // 创建测试用的地图数据
    nav_msgs::OccupancyGrid createTestMap() {
        nav_msgs::OccupancyGrid map;
        map.header.frame_id = "map";
        map.header.stamp = ros::Time::now();
        
        // 设置地图属性
        map.info.resolution = 0.05;  // 5cm分辨率
        map.info.width = 100;        // 5米宽
        map.info.height = 100;       // 5米高
        map.info.origin.position.x = -2.5;
        map.info.origin.position.y = -2.5;
        
        // 创建地图数据
        map.data.resize(map.info.width * map.info.height, 0);
        
        // 添加一些障碍物（边界墙）
        for (unsigned int i = 0; i < map.info.width; ++i) {
            // 上下边界
            map.data[i] = 100;
            map.data[i + (map.info.height-1) * map.info.width] = 100;
        }
        for (unsigned int i = 0; i < map.info.height; ++i) {
            // 左右边界
            map.data[i * map.info.width] = 100;
            map.data[i * map.info.width + map.info.width-1] = 100;
        }
        
        return map;
    }

    // 创建测试用的里程计数据
    nav_msgs::Odometry createTestOdom() {
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.child_frame_id = "base_link";
        
        // 设置位置
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;
        
        // 设置朝向（四元数表示）
        odom.pose.pose.orientation.w = 1.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
        
        return odom;
    }

    // 创建测试用的激光扫描数据
    sensor_msgs::LaserScan createTestScan() {
        sensor_msgs::LaserScan scan;
        scan.header.frame_id = "base_laser";
        scan.header.stamp = ros::Time::now();
        
        // 设置激光扫描参数
        scan.angle_min = -M_PI/2;
        scan.angle_max = M_PI/2;
        scan.angle_increment = M_PI/180;  // 1度分辨率
        scan.time_increment = 0.0;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        
        // 创建扫描数据（180个点）
        int num_readings = 180;
        scan.ranges.resize(num_readings);
        for (int i = 0; i < num_readings; ++i) {
            scan.ranges[i] = 1.0;  // 1米距离
        }
        
        return scan;
    }

    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher map_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher scan_pub_;
};

// 测试地图显示功能
TEST_F(MapViewTest, TestMapDisplay) {
    // 创建主窗口
    int argc = 0;
    char** argv = nullptr;
    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    // 发布测试数据
    map_pub_.publish(createTestMap());
    odom_pub_.publish(createTestOdom());
    scan_pub_.publish(createTestScan());

    // 处理ROS消息和Qt事件
    ros::Duration(1.0).sleep();  // 等待消息处理
    QApplication::processEvents();

    // 验证窗口是否正确显示
    EXPECT_TRUE(window.isVisible());

    // 继续发布更新的数据进行测试
    nav_msgs::Odometry odom = createTestOdom();
    for (int i = 0; i < 10; ++i) {
        // 更新机器人位置
        odom.pose.pose.position.x += 0.1;
        odom.pose.pose.position.y += 0.1;
        odom_pub_.publish(odom);

        // 处理消息和事件
        ros::Duration(0.1).sleep();
        QApplication::processEvents();
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 