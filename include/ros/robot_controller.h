#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <QObject>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map_server/image_loader.h>
#include <string>
#include <memory>
#include <exception>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotController : public QObject {
    Q_OBJECT

public:
    explicit RobotController(QObject* parent = nullptr);
    ~RobotController();

    // 导航控制
    bool setNavigationGoal(double x, double y, double theta);
    void cancelNavigation();
    bool isNavigating() const;
    bool setNavigationMode(int mode);
    int getNavigationMode() const;

    // 建图控制
    bool startMapping();
    bool stopMapping();
    bool saveMap(const std::string& filename);
    bool loadMap(const std::string& filename);

    // 位姿设置
    bool setInitialPose(double x, double y, double theta);

    // 代价地图操作
    bool updateCostmap();

    // 速度控制
    void publishVelocity(double linear, double angular);
    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);
    double getMaxLinearVelocity() const { return max_linear_velocity_; }
    double getMaxAngularVelocity() const { return max_angular_velocity_; }
    double getCurrentLinearVelocity() const { return current_linear_velocity_; }
    double getCurrentAngularVelocity() const { return current_angular_velocity_; }

    // 导航控制
    void startNavigation();
    void stopNavigation();

    // 状态查询
    double getBatteryLevel() const { return battery_level_; }
    int getWifiStrength() const { return wifi_strength_; }
    QString getStatus() const { return status_; }

signals:
    // 数据更新信号
    void mapUpdated(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void odomUpdated(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void scanUpdated(const std::shared_ptr<sensor_msgs::LaserScan>& scan);
    void statusChanged(const QString& status);

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    
    // ROS发布器
    ros::Publisher cmd_vel_pub_;
    ros::Publisher initial_pose_pub_;
    
    // ROS订阅器
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

    // Action客户端
    std::shared_ptr<MoveBaseClient> move_base_client_;

    // 状态变量
    bool is_navigating_;
    bool is_mapping_;
    int navigation_mode_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double current_linear_velocity_ = 0.0;
    double current_angular_velocity_ = 0.0;

    // 状态信息
    double battery_level_ = 100.0;  // 电池电量（百分比）
    int wifi_strength_ = 100;      // WiFi信号强度（百分比）
    QString status_ = "就绪";      // 机器人状态

    // ROS回调函数
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Action回调函数
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
};

#endif // ROBOT_CONTROLLER_H 