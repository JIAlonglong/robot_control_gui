#ifndef ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H
#define ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H

#include <QObject>
#include <QString>
#include <QTimer>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// 导航状态枚举
enum class NavigationState {
    IDLE,           // 空闲
    PLANNING,       // 规划中
    MOVING,         // 移动中
    ROTATING,       // 旋转中
    ADJUSTING,      // 微调中
    SUCCEEDED,      // 成功到达
    FAILED,         // 导航失败
    CANCELED        // 已取消
};

class RobotController : public QObject {
    Q_OBJECT

public:
    explicit RobotController(QObject* parent = nullptr);
    ~RobotController() override;

    // 速度控制
    void setLinearVelocity(double linear);
    void setAngularVelocity(double angular);
    void stop();
    void publishVelocity(double linear, double angular);
    void setMaxLinearVelocity(double max_linear);
    void setMaxAngularVelocity(double max_angular);
    double getMaxLinearVelocity() const { return max_linear_velocity_; }
    double getMaxAngularVelocity() const { return max_angular_velocity_; }
    double getCurrentLinearVelocity() const { return linear_velocity_; }
    double getCurrentAngularVelocity() const { return angular_velocity_; }

    // 状态查询
    double getBatteryPercentage() const { return battery_percentage_; }
    double getBatteryVoltage() const { return battery_voltage_; }
    double getBatteryCurrent() const { return battery_current_; }
    double getBatteryTemperature() const { return battery_temperature_; }
    QString getMotorStatus() const { return motor_status_; }
    geometry_msgs::Pose getCurrentPose() const { return current_pose_; }
    geometry_msgs::Pose getCurrentAmclPose() const { return current_amcl_pose_; }
    sensor_msgs::LaserScan getCurrentScan() const { return current_scan_; }
    bool isInitialized() const { return is_initialized_; }
    NavigationState getNavigationState() const { return nav_state_; }
    double getNavigationProgress() const { return navigation_progress_; }
    QString getNavigationStatusText() const { return nav_status_text_; }

    // 导航相关
    void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void setNavigationGoal(const geometry_msgs::PoseStamped& goal);
    void startNavigation();
    void pauseNavigation();
    void stopNavigation();
    void cancelNavigation();

    // 定位相关
    void startGlobalLocalization();
    void cancelGlobalLocalization();

    // 地图相关
    void startMapping();
    void stopMapping();
    void saveMap(const QString& filename);
    void loadMap(const QString& filename);

    // 导航参数设置
    void setYawTolerance(double value);
    void setInflationRadius(double value);
    void setTransformTolerance(double value);
    void setPlannerFrequency(double value);
    void setControllerFrequency(double value);
    void setGlobalCostmapUpdateFrequency(double value);
    void setLocalCostmapUpdateFrequency(double value);
    void setPlannedPathBias(double value);
    void setRecoveryBehaviorEnabled(bool enabled);
    void setClearingRotationAllowed(bool allowed);
    void setNavigationMode(int mode);

    // 参数获取
    double getYawTolerance() const { return yaw_tolerance_; }
    double getInflationRadius() const { return inflation_radius_; }
    double getTransformTolerance() const { return transform_tolerance_; }
    double getPlannerFrequency() const { return planner_frequency_; }
    double getControllerFrequency() const { return controller_frequency_; }
    double getGlobalCostmapUpdateFrequency() const { return global_costmap_update_frequency_; }
    double getLocalCostmapUpdateFrequency() const { return local_costmap_update_frequency_; }
    double getPlannedPathBias() const { return planned_path_bias_; }
    bool isRecoveryBehaviorEnabled() const { return recovery_behavior_enabled_; }
    bool isClearingRotationAllowed() const { return clearing_rotation_allowed_; }
    int getNavigationMode() const { return navigation_mode_; }

    // 网络设置
    bool testConnection(const std::string& master_uri);
    void setMasterURI(const std::string& uri);
    void setHostname(const std::string& hostname);

    // 机器人配置
    void setRobotModel(const std::string& model);
    void setSerialPort(const std::string& port);
    void setBaudrate(int baudrate);

Q_SIGNALS:
    // 速度状态信号
    void velocityChanged(double linear, double angular);
    
    // 电池状态信号
    void batteryStateChanged(const sensor_msgs::BatteryState& status);
    
    // 诊断信息信号
    void diagnosticsUpdated(const diagnostic_msgs::DiagnosticArray& diagnostics);
    
    // 激光扫描信号
    void laserScanUpdated(const sensor_msgs::LaserScan& scan);
    
    // 位姿更新信号
    void poseUpdated(const geometry_msgs::PoseWithCovariance& pose);

    // 导航相关信号
    void navigationStateChanged(NavigationState state);
    void navigationProgressChanged(double progress);
    void navigationModeChanged(int mode);
    void distanceToGoalChanged(double distance);
    void estimatedTimeToGoalChanged(double time);
    void navigationFeedback(const QString& status);

    // 定位相关信号
    void localizationStateChanged(bool active);
    void localizationStatusChanged(const QString& status);
    void localizationProgressChanged(double progress);

    // 地图相关信号
    void mapUpdated(const nav_msgs::OccupancyGrid& map);
    void mappingStateChanged(bool active);
    void mappingProgressChanged(double progress);

private:
    void setupPublishers();
    void setupSubscribers();
    void cleanup();
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber diagnostics_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber map_sub_;
    
    // 服务客户端
    ros::ServiceClient global_localization_client_;
    ros::ServiceClient clear_costmaps_client_;
    
    // Action 客户端
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;

    // 速度相关
    double linear_velocity_{0.0};
    double angular_velocity_{0.0};
    double max_linear_velocity_{1.0};
    double max_angular_velocity_{1.0};

    // 状态相关
    double battery_percentage_{0.0};
    double battery_voltage_{0.0};
    double battery_current_{0.0};
    double battery_temperature_{0.0};
    QString motor_status_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose current_amcl_pose_;
    sensor_msgs::LaserScan current_scan_;
    bool is_initialized_{false};

    // 导航相关
    bool is_navigating_{false};
    NavigationState nav_state_{NavigationState::IDLE};
    double navigation_progress_{0.0};
    double distance_to_goal_{0.0};
    double estimated_time_to_goal_{0.0};
    QString nav_status_text_{"就绪"};
    geometry_msgs::PoseStamped current_goal_;

    // 定位相关
    bool is_localizing_{false};
    double localization_progress_{0.0};

    // 地图相关
    bool is_mapping_{false};
    double mapping_progress_{0.0};

    // 导航参数
    double yaw_tolerance_{0.1};
    double inflation_radius_{0.55};
    double transform_tolerance_{0.2};
    double planner_frequency_{5.0};
    double controller_frequency_{10.0};
    double global_costmap_update_frequency_{5.0};
    double local_costmap_update_frequency_{5.0};
    double planned_path_bias_{0.5};
    bool recovery_behavior_enabled_{true};
    bool clearing_rotation_allowed_{true};
    int navigation_mode_{0};

    // 辅助函数
    void updateNavigationProgress(const geometry_msgs::PoseStamped& current_pose);
    double calculateDistance(const geometry_msgs::PoseStamped& pose1,
                           const geometry_msgs::PoseStamped& pose2);
    void updateNavigationState(NavigationState state, const QString& status = QString());
};

#endif // ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H 