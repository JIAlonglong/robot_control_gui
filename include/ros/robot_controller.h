#ifndef ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H
#define ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H

#include <QObject>
#include <QString>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <tf/transform_listener.h>

class RobotController : public QObject {
    Q_OBJECT
public:
    enum class NavigationState {
        IDLE,
        ACTIVE,
        PAUSED,
        STOPPED,
        CANCELLED,
        SUCCEEDED,
        FAILED
    };
    Q_ENUM(NavigationState)

    enum class InteractionMode {
        NONE,
        SET_INITIAL_POSE,
        SET_GOAL
    };
    Q_ENUM(InteractionMode)

    explicit RobotController(QObject* parent = nullptr);
    ~RobotController() override;

    bool isInitialized() const { return is_initialized_; }
    bool isNavigating() const { return is_navigating_; }
    bool isLocalized() const { return is_localized_; }
    bool isMapping() const { return is_mapping_; }

    void setMasterURI(const QString& uri);
    void setHostname(const QString& hostname);

    void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void setNavigationGoal(const geometry_msgs::PoseStamped& goal);
    void enableInitialPoseSetting(bool enable);
    void enableGoalSetting(bool enable);

    void startNavigation();
    void pauseNavigation();
    void stopNavigation();
    void cancelNavigation();

    void startGlobalLocalization();
    void cancelGlobalLocalization();
    void startAutoLocalization();

    void startMapping();
    void stopMapping();
    void saveMap(const QString& filename);
    void loadMap(const QString& filename);

    void setLinearVelocity(double linear);
    void setAngularVelocity(double angular);
    void setMaxLinearVelocity(double max_linear);
    void setMaxAngularVelocity(double max_angular);
    void emergencyStop();

    double getLinearVelocity() const { return current_linear_velocity_; }
    double getAngularVelocity() const { return current_angular_velocity_; }
    double getMaxLinearVelocity() const { return max_linear_velocity_; }
    double getMaxAngularVelocity() const { return max_angular_velocity_; }

    double getBatteryPercentage() const { return battery_percentage_; }
    double getBatteryVoltage() const { return battery_voltage_; }
    double getBatteryCurrent() const { return battery_current_; }
    double getBatteryTemperature() const { return battery_temperature_; }
    QString getMotorStatus() const { return motor_status_; }

    void stop();
    void publishVelocity(double linear, double angular);
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
    bool testConnection(const std::string& master_uri);
    void setRobotModel(const std::string& model);
    void setSerialPort(const std::string& port);
    void setBaudrate(int baudrate);

    double getCurrentLinearVelocity() const { return linear_velocity_; }
    double getCurrentAngularVelocity() const { return angular_velocity_; }

    void setGlobalPlanner(const QString& planner_name);
    void setLocalPlanner(const QString& planner_name);
    std::vector<QString> getAvailableGlobalPlanners() const;
    std::vector<QString> getAvailableLocalPlanners() const;
    QString getCurrentGlobalPlanner() const { return current_global_planner_; }
    QString getCurrentLocalPlanner() const { return current_local_planner_; }

signals:
    void velocityChanged(double linear, double angular);
    void batteryStateChanged(const sensor_msgs::BatteryState& state);
    void localizationStateChanged(bool is_localized);
    void localizationProgressChanged(double progress);
    void localizationStatusChanged(const QString& status);
    void navigationStateChanged(NavigationState state);
    void navigationProgressChanged(double progress);
    void navigationStatusChanged(const QString& status);
    void distanceToGoalChanged(double distance);
    void estimatedTimeToGoalChanged(double time);
    void mapUpdated(const nav_msgs::OccupancyGrid& map);
    void mappingStateChanged(bool is_mapping);
    void mappingProgressChanged(double progress);
    void navigationModeChanged(int mode);
    void poseUpdated(const geometry_msgs::PoseWithCovariance& pose);
    void laserScanUpdated(const sensor_msgs::LaserScan& scan);
    void diagnosticsUpdated(const diagnostic_msgs::DiagnosticArray& array);
    void goalDisplayEnabled(bool enabled);
    void globalPlannerChanged(const QString& planner_name);
    void localPlannerChanged(const QString& planner_name);

private:
    void cleanup();
    void setupPublishers();
    void setupSubscribers();
    void setupSafety();
    void monitorLocalization(const ros::TimerEvent&);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    void navigationDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const move_base_msgs::MoveBaseResultConstPtr& result);
    void navigationActiveCallback();
    void navigationFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    ros::NodeHandle nh_;
    ros::Publisher global_localization_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Publisher tool_manager_pub_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber diagnostics_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber goal_sub_;

    ros::ServiceClient global_localization_client_;
    ros::ServiceClient clear_costmaps_client_;
    ros::Timer localization_monitor_timer_;

    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

    geometry_msgs::PoseWithCovarianceStamped current_amcl_pose_;
    boost::array<double, 36> current_pose_cov_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::PoseStamped current_goal_;
    sensor_msgs::LaserScan current_scan_;

    bool is_initialized_{false};
    bool is_navigating_{false};
    bool is_localized_{false};
    bool is_mapping_{false};
    bool is_localizing_{false};
    bool is_obstacle_detected_{false};

    double safety_distance_{0.3};
    double current_linear_velocity_{0.0};
    double current_angular_velocity_{0.0};
    double max_linear_velocity_{1.0};
    double max_angular_velocity_{2.0};
    double linear_velocity_{0.0};
    double angular_velocity_{0.0};

    double battery_percentage_{0.0};
    double battery_voltage_{0.0};
    double battery_current_{0.0};
    double battery_temperature_{0.0};
    QString motor_status_;

    double initial_distance_{0.0};
    double distance_to_goal_{0.0};
    double estimated_time_to_goal_{0.0};
    double navigation_progress_{0.0};
    double localization_progress_{0.0};
    double mapping_progress_{0.0};

    double yaw_tolerance_{0.1};
    double inflation_radius_{0.3};
    double transform_tolerance_{0.2};
    double planner_frequency_{5.0};
    double controller_frequency_{10.0};
    double global_costmap_update_frequency_{5.0};
    double local_costmap_update_frequency_{5.0};
    double planned_path_bias_{0.8};
    bool recovery_behavior_enabled_{true};
    bool clearing_rotation_allowed_{true};
    int navigation_mode_{0};
    
    RobotController::InteractionMode interaction_mode_{RobotController::InteractionMode::NONE};

    QString current_global_planner_{"navfn/NavfnROS"};
    QString current_local_planner_{"base_local_planner/TrajectoryPlannerROS"};

    // TF相关
    tf::TransformListener tf_listener_;
    
    // 自动定位相关
    void publishLocalizationMarkers();
    ros::Publisher marker_pub_;
    double auto_localization_radius_;
    geometry_msgs::Point localization_center_;
    double current_rotation_speed_;
    double current_linear_speed_;
    ros::Time last_direction_change_;

    // 安全相关
    bool left_space_larger_{true};
};

#endif // ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H 