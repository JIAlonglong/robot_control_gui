#pragma once

#ifndef MAPPING_CONTROLLER_H
#define MAPPING_CONTROLLER_H

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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <memory>
#include <geometry_msgs/Point.h>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class RobotController;

class MappingController : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool isMapping READ isMapping NOTIFY mappingStateChanged)
    Q_PROPERTY(bool isAutoMapping READ isAutoMapping NOTIFY autoMappingStateChanged)
    Q_PROPERTY(bool isPaused READ isPaused NOTIFY mappingStateChanged)
    Q_PROPERTY(QString mappingState READ mappingState NOTIFY mappingStateChanged)
    Q_PROPERTY(double mappingProgress READ mappingProgress NOTIFY mappingProgressChanged)
    Q_PROPERTY(QString mappingStatus READ mappingStatus NOTIFY mappingStatusChanged)

public:
    explicit MappingController(QObject* parent = nullptr);
    ~MappingController() override;

    // 状态查询
    bool isMapping() const;
    bool isAutoMapping() const;
    bool isPaused() const;
    QString mappingState() const;
    double mappingProgress() const;
    QString mappingStatus() const;

    // 建图控制
    void startMapping();
    void stopMapping();
    void pauseMapping();
    void resumeMapping();
    void saveMap(const QString& filename);
    void loadMap(const QString& filename);
    void clearMap();

    // 自动建图控制
    bool startAutoMapping(const QString& method, double resolution);
    void stopAutoMapping();
    void pauseAutoMapping();
    void resumeAutoMapping();

    // 参数设置
    void setMappingMethod(const QString& method);
    void setUpdateRate(int rate);
    void setRadius(double radius);
    void setFrontierSize(double size);
    void setMaxTime(int time);
    void setReturnHome(bool enabled);

    // 建图相关方法
    bool startMapping(const QString& method, const QString& map_name,
                     const QString& map_path, double resolution,
                     bool update_in_real_time, const QVariantMap& params);
    void stopMapping(const QString& method);
    void pauseMapping(const QString& method);
    void resumeMapping(const QString& method);
    void saveMap();
    QString getCurrentMapPath() const { return current_map_path_; }

    // 建图监控和诊断方法
    void monitorMappingStatus();
    void updateMappingStatistics();
    void checkResourceUsage();
    void optimizeResourceUsage();
    void checkMappingQuality();
    void checkMappingProgress();
    void emitDiagnosticInfo();
    bool checkSensorsStatus();
    bool checkSystemResources();
    bool checkDiskSpace(const QString& path);
    QString formatTimeRemaining(qint64 seconds);

    void setMapResolution(double resolution);
    void setUpdateInterval(int interval);

    void setMethod(const QString& method);
    void setUpdateRate(int rate);
    void setMaxRange(double range);
    void setMinRange(double range);
    void setAngleMin(double angle);
    void setAngleMax(double angle);
    void setAngleIncrement(double increment);
    void setTimeIncrement(double increment);
    void setScanTime(double time);
    void setRangeMin(double range);
    void setRangeMax(double range);
    void setIntensitiesEnabled(bool enabled);

    // 自动建图参数设置
    void setExplorationStrategy(const QString& strategy);
    void setExplorationRadius(double radius);
    void setMinFrontierSize(double size);
    void setMaxExplorationTime(int minutes);
    void setReturnHomeEnabled(bool enabled);
    
    // 获取自动建图状态
    QString explorationStrategy() const { return exploration_strategy_; }
    double explorationProgress() const { return exploration_progress_; }

Q_SIGNALS:
    void mappingStateChanged(const QString& state);
    void autoMappingStateChanged(bool is_auto_mapping);
    void mappingProgressChanged(double progress);
    void mappingFailed(const QString& error);
    void mappingCompleted();
    void mapSaved(const QString& filename);
    void mapLoaded(const QString& filename);
    void mappingStatusChanged(const QString& status);
    void mapUpdated(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void updateIntervalChanged(int interval);
    void mapResolutionChanged(double resolution);
    void mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void scanReceived(const sensor_msgs::LaserScan::ConstPtr& scan);
    void error(const QString& message);
    void mappingStatus(const QString& status);
    void mappingProgress(double progress);

    // 添加自动建图相关信号
    void autoMappingStarted();
    void autoMappingStopped();
    void autoMappingPaused();
    void autoMappingResumed();
    void explorationProgressChanged(double progress);
    void frontierDetected(const geometry_msgs::Point& frontier);
    void newAreaExplored(double area);
    void returnHomeStarted();
    void returnHomeCompleted();
    void explorationCompleted();

    void mappingStarted(const QString& method);
    void mappingFinished();
    void mappingCancelled();
    void mappingStatusUpdated(const QString& status);
    void mappingProgressUpdated(double progress);

private Q_SLOTS:
    void handleMappingProcessOutput();
    void handleMappingProcessError(QProcess::ProcessError error);
    void onMappingStateChanged(const QString& state);
    void onMappingProgressChanged(double progress);
    void onMappingStatusChanged(const QString& status);
    void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void onScanReceived(const sensor_msgs::LaserScan::ConstPtr& scan);
    void onError(const QString& message);
    void onTimeout();

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void cleanupMappingProcess();
    void adjustGmappingParams(const QString& output);
    void adjustCartographerParams(const QString& output);
    void adjustHectorParams(const QString& output);
    void adjustRtabmapParams(const QString& output);
    void analyzeAndAdjustParams(const QString& error);
    void restartMapping();
    void attemptRecovery();
    void handleRecoveryFailure();
    void attemptRecoveryForStartFailure();
    void attemptRecoveryForCrash();
    void attemptRecoveryForTimeout();
    void attemptRecoveryForIOError();
    void reduceMemoryUsage();
    void backupAndRestartMapping();
    void cleanupOldMaps();
    void autoSaveMap();
    void logError();

    // 添加自动建图相关私有成员
    bool is_auto_mapping_{false};
    QString exploration_strategy_{"frontier"};
    double exploration_radius_{5.0};
    double min_frontier_size_{1.0};
    int max_exploration_time_{30};  // 分钟
    bool return_home_enabled_{true};
    double exploration_progress_{0.0};
    geometry_msgs::Point home_position_;
    std::vector<geometry_msgs::Point> frontiers_;
    
    // 添加自动建图相关私有方法
    void detectFrontiers();
    void selectNextFrontier();
    void navigateToFrontier(const geometry_msgs::Point& frontier);
    void updateExplorationProgress();
    void checkExplorationTimeout();
    void returnToHome();
    bool isAreaFullyExplored();
    void processLaserScan(const sensor_msgs::LaserScan& scan);
    void updateFrontiers();
    double calculateExploredArea();
    bool isValidFrontier(const geometry_msgs::Point& point);
    void monitorExplorationProgress();

    std::shared_ptr<RobotController> robot_controller_;
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    QProcess* mapping_process_{nullptr};
    QString current_map_path_;
    QString current_map_name_;
    QString current_map_method_;
    double map_resolution_{0.05};
    bool is_mapping_{false};
    bool update_map_in_real_time_{true};
    double mapping_progress_{0.0};
    QVariantMap mapping_params_;
    QDateTime mapping_start_time_;
    QMap<QString, QVariant> mapping_stats_;
    int consecutive_high_resource_usage_{0};
    bool is_in_recovery_{false};
    int retry_count_{0};
    QString last_error_type_;
    QDateTime last_error_time_;
    QTimer* recovery_timer_{nullptr};
    static constexpr int MAX_RETRIES{3};
    static constexpr int MAX_HIGH_RESOURCE_COUNT{5};
    int update_interval_{1000};  // 默认更新间隔为1秒
    QTimer* update_timer_{nullptr};
    QString mapping_state_;
    QString mapping_status_;
};

#endif // MAPPING_CONTROLLER_H 