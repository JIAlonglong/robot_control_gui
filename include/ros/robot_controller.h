#pragma once

#ifndef ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H
#define ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H

// Qt头文件
#include <QObject>
#include <QString>
#include <QProcess>
#include <QVariantMap>
#include <QTimer>
#include <QDateTime>
#include <QMap>
#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <QThread>
#include <QStorageInfo>
#include <QDebug>
#include <QMutex>
#include <memory>
#include <QVariant>
#include <array>

// ROS头文件
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
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <std_msgs/Float64.h>

class RobotController : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionStateChanged)
    Q_PROPERTY(QString robotState READ robotState NOTIFY robotStateChanged)
    Q_PROPERTY(double batteryLevel READ batteryLevel NOTIFY batteryLevelChanged)
    Q_PROPERTY(double batteryVoltage READ batteryVoltage NOTIFY batteryVoltageChanged)
    Q_PROPERTY(double batteryCurrent READ batteryCurrent NOTIFY batteryCurrentChanged)
    Q_PROPERTY(double batteryTemperature READ batteryTemperature NOTIFY batteryTemperatureChanged)
    Q_PROPERTY(QString batteryStatus READ batteryStatus NOTIFY batteryStatusChanged)
    Q_PROPERTY(double linearVelocity READ linearVelocity NOTIFY linearVelocityChanged)
    Q_PROPERTY(double angularVelocity READ angularVelocity NOTIFY angularVelocityChanged)
    Q_PROPERTY(QString navigationState READ navigationState NOTIFY navigationStateChanged)
    Q_PROPERTY(QString localizationState READ localizationState NOTIFY localizationStateChanged)
    Q_PROPERTY(QString mappingState READ mappingState NOTIFY mappingStateChanged)

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
        SET_GOAL,
        SET_NAVIGATION_GOAL
    };
    Q_ENUM(InteractionMode)

    enum class MappingMethod {
        SLAM_TOOLBOX,
        CARTOGRAPHER,
        GMAPPING
    };
    Q_ENUM(MappingMethod)

    explicit RobotController(QObject* parent = nullptr);
    ~RobotController() override;

    // 初始化和清理
    bool initialize();
    void cleanup();

    // 导航控制
    void startNavigation();
    void pauseNavigation();
    void stopNavigation();
    void resumeNavigation();
    void emergencyStop();

    // 导航设置
    void setNavigationMode(int mode);
    void setGlobalPlanner(const QString& planner_name);
    void setLocalPlanner(const QString& planner_name);

    // 建图控制
    void setMappingMethod(MappingMethod method);
    void startMapping();
    void stopMapping();
    void saveMap(const QString& filename);

    // 状态查询
    QString batteryStatus() const;
    double batteryLevel() const;
    double batteryVoltage() const;
    double batteryCurrent() const;
    double batteryTemperature() const;

    // 连接相关方法
    bool testConnection(const std::string& master_uri);
    bool isInitialized() const;
    bool isConnected() const;

    // 基本状态查询
    bool isNavigating() const;
    bool isLocalized() const;
    geometry_msgs::Pose getCurrentPose() const;

    // 基本设置
    void setMasterURI(const QString& uri);
    void setHostname(const QString& hostname);

    // 导航相关方法
    void setNavigationGoal(const geometry_msgs::PoseStamped& goal);
    void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void sendGoal(const geometry_msgs::PoseStamped& goal);
    void startAutoLocalization();
    void stopAutoLocalization();
    void startGlobalLocalization();
    void cancelGlobalLocalization();

    // 速度控制
    void setLinearVelocity(double linear);
    void setAngularVelocity(double angular);
    void setVelocity(double linear, double angular);
    void setMaxLinearVelocity(double max_linear);
    void setMaxAngularVelocity(double max_angular);
    void stop();
    void cancelGoal();

    // 获取速度
    double getLinearVelocity() const;
    double getAngularVelocity() const;
    double getMaxLinearVelocity() const;
    double getMaxAngularVelocity() const;

    // 电池状态
    double getBatteryPercentage() const;
    QString getMotorStatus() const;

    // 参数设置
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
    void setRobotModel(const std::string& model);
    void setSerialPort(const std::string& port);
    void setBaudrate(int baudrate);

    // 速度相关
    double getCurrentLinearVelocity() const;
    double getCurrentAngularVelocity() const;
    void setPlanner(const QString& planner);

    // 地图相关
    void loadMap(const QString& filename);

    void setMaxSpeed(double speed);
    void setMaxAcceleration(double accel);
    void setSafetyDistance(double distance);
    void setUpdateFrequency(int frequency);
    void setAutoConnect(bool enabled);
    void setDebugMode(bool enabled);
    void setPlannerFrequency(int frequency);

    void setPlanner(const std::string& planner);
    void setResolution(double resolution);
    void setUpdateRate(int rate);
    void setMaxRange(double range);

    // 新增方法
    void setMaxAngularSpeed(double value);
    void setControlMode(int mode);
    void setPlanningFrequency(int value);
    void setMapSize(int value);
    void setGoalTolerance(double tolerance);
    void setPlannerType(const QString& type);
    void onConnectionTimeout();

    // 地图相关
    void updateMap(const QString& map_topic);
    void updateRobotPose(const QString& pose_topic);
    void updateLaserScan(const QString& scan_topic);
    void updatePath(const QString& path_topic);

    // 传感器相关
    double getSensorValue(const QString& name) const;
    bool isObstacleDetected() const;

    // 获取当前状态
    sensor_msgs::LaserScan getCurrentLaserScan() const;

    QString robotState() const;
    QString navigationState() const;
    QString localizationState() const;
    QString mappingState() const;

    // 添加缺失的函数声明
    std::vector<QString> getAvailableGlobalPlanners() const;
    std::vector<QString> getAvailableLocalPlanners() const;
    QString getCurrentGlobalPlanner() const;
    QString getCurrentLocalPlanner() const;

    // 修改函数声明
    void saveMappingResults();
    void enableInitialPose(bool enable);

    void move(double linear_x, double linear_y, double angular_z);
    void navigateTo(double x, double y, double theta, double tolerance = 0.1);
    void pauseMapping();
    void resumeMapping();
    QVariantMap properties() const;

    void stopRobot();
    void cancelNavigation();
    void clearCostmaps();
    bool isLidarConnected() const;
    bool isImuConnected() const;
    bool isOdomConnected() const;

    void enableAutoLocalization(bool enable);

public Q_SLOTS:
    void connectToROS();
    void disconnect();
    void setRobotModel(const QString& model);
    void setSerialPort(const QString& port);
    void setBaudRate(int baud_rate);
    void setProperty(const QString& name, const QVariant& value);
    void setBatteryStatus(const QString& status);

Q_SIGNALS:
    void connectionStateChanged(bool connected);
    void robotStateChanged(const QString& state);
    void batteryLevelChanged(double level);
    void batteryVoltageChanged(double voltage);
    void batteryCurrentChanged(double current);
    void batteryTemperatureChanged(double temperature);
    void batteryStatusChanged(const QString& status);
    void linearVelocityChanged(double velocity);
    void angularVelocityChanged(double velocity);
    void navigationStateChanged(const QString& state);
    void localizationStateChanged(const QString& state);
    void mappingStateChanged(const QString& state);
    void navigationProgress(double progress);
    void navigationCompleted(bool success);
    void obstacleDetected(bool detected);
    void scanUpdated(const sensor_msgs::LaserScan::ConstPtr& scan);
    void poseUpdated(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
    void mapUpdated(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void pathUpdated(const nav_msgs::Path::ConstPtr& path);
    void velocityUpdated(double linear, double angular);
    void batteryStateUpdated(double level, double voltage, double current, double temperature);
    void error(const QString& message);
    void connected();
    void disconnected();
    void navigationStarted();
    void navigationCancelled();
    void navigationStopped();
    void costmapsCleared();
    void robotModelChanged(const QString& model);
    void serialPortChanged(const QString& port);
    void baudrateChanged(int baudrate);
    void autoConnectChanged(bool enabled);
    void debugModeChanged(bool enabled);
    void maxSpeedChanged(double speed);
    void maxAngularSpeedChanged(double speed);
    void maxAccelerationChanged(double accel);
    void safetyDistanceChanged(double distance);
    void updateFrequencyChanged(int freq);
    void plannerTypeChanged(const QString& type);
    void planningFrequencyChanged(int freq);
    void goalToleranceChanged(double tolerance);
    void laserScanUpdated(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    void publishVelocity();
    void navigationDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const move_base_msgs::MoveBaseResultConstPtr& result);
    void navigationActiveCallback();
    void navigationFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void mapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    struct Private;
    std::unique_ptr<Private> d_;

    bool auto_localization_enabled_{false};
    ros::Subscriber localization_quality_sub_;
};

#endif // ROBOT_CONTROL_GUI_ROBOT_CONTROLLER_H