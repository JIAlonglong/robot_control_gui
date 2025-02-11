/**
 * Copyright (c) 2024 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file mapping_controller.cpp
 * @brief 建图控制器类的实现,负责机器人建图功能
 * @author JIAlonglong
 */

#include "mapping_controller.h"
#include <QDebug>
#include <QDir>
#include <QProcess>
#include <QTimer>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <QStorageInfo>

struct MappingController::Private {
    ros::NodeHandle nh;
    ros::Publisher  map_pub;
    ros::Subscriber scan_sub;
    ros::Subscriber map_sub;
    ros::ServiceClient save_map_client;
    
    QString mapping_state{"未启动"};
    QString mapping_status{"就绪"};
    QString mapping_method{"gmapping"};
    double  mapping_progress{0.0};
    bool    is_mapping{false};
    bool    is_auto_mapping{false};
    bool    is_paused{false};
    
    QProcess* mapping_process{nullptr};
    QTimer*   monitor_timer{nullptr};
    QString   current_map_path;
    
    // 建图参数
    double resolution{0.05};
    int    update_rate{10};
    double max_range{30.0};
    double min_range{0.1};
    double angle_min{-M_PI};
    double angle_max{M_PI};
    double angle_increment{M_PI/360.0};
    double time_increment{0.0};
    double scan_time{0.1};
    bool   intensities_enabled{false};
    
    // 自动建图参数
    QString exploration_strategy{"frontier"};
    double  exploration_radius{3.0};
    double  min_frontier_size{0.5};
    int     max_exploration_time{30};  // 分钟
    bool    return_home_enabled{true};
};

MappingController::MappingController(QObject* parent)
    : QObject(parent), d_(new Private)
{
    // 初始化ROS通信
    d_->map_pub = d_->nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    d_->scan_sub = d_->nh.subscribe("scan", 1, &MappingController::scanCallback, this);
    d_->map_sub = d_->nh.subscribe("map", 1, &MappingController::mapCallback, this);
    d_->save_map_client = d_->nh.serviceClient<nav_msgs::GetMap>("static_map");
    
    // 创建监控定时器
    d_->monitor_timer = new QTimer(this);
    connect(d_->monitor_timer, &QTimer::timeout, this, &MappingController::monitorMappingStatus);
    d_->monitor_timer->setInterval(1000);  // 1Hz
}

MappingController::~MappingController()
{
    if (d_->is_mapping) {
        stopMapping();
    }
    delete d_->monitor_timer;
}

void MappingController::startMapping()
{
    if (d_->is_mapping) {
        return;
    }
    
    // 检查传感器状态
    if (!checkSensorsStatus()) {
        emit error("传感器状态异常,无法开始建图");
        return;
    }
    
    // 检查系统资源
    if (!checkSystemResources()) {
        emit error("系统资源不足,无法开始建图");
        return;
    }
    
    // 启动建图进程
    d_->mapping_process = new QProcess(this);
    connect(d_->mapping_process, &QProcess::readyReadStandardOutput,
            this, &MappingController::handleMappingProcessOutput);
    connect(d_->mapping_process, QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
            this, &MappingController::handleMappingProcessError);
    
    QString cmd = QString("rosrun gmapping slam_gmapping ");
    cmd += QString("scan:=scan ");
    cmd += QString("_base_frame:=base_footprint ");
    cmd += QString("_map_update_interval:=%1 ").arg(d_->update_rate);
    cmd += QString("_maxUrange:=%1 ").arg(d_->max_range);
    cmd += QString("_minimumScore:=50 ");
    cmd += QString("_linearUpdate:=0.2 ");
    cmd += QString("_angularUpdate:=0.2 ");
    cmd += QString("_particles:=30 ");
    
    d_->mapping_process->start(cmd);
    
    if (!d_->mapping_process->waitForStarted()) {
        emit error("建图进程启动失败");
        delete d_->mapping_process;
        d_->mapping_process = nullptr;
        return;
    }
    
    d_->is_mapping = true;
    d_->mapping_state = "建图中";
    d_->mapping_progress = 0.0;
    d_->monitor_timer->start();
    
    emit mappingStateChanged(d_->mapping_state);
    emit mappingStarted();
}

void MappingController::stopMapping()
{
    if (!d_->is_mapping) {
        return;
    }
    
    if (d_->mapping_process) {
        d_->mapping_process->terminate();
        if (!d_->mapping_process->waitForFinished(3000)) {
            d_->mapping_process->kill();
        }
        delete d_->mapping_process;
        d_->mapping_process = nullptr;
    }
    
    d_->monitor_timer->stop();
    d_->is_mapping = false;
    d_->is_paused = false;
    d_->mapping_state = "已停止";
    d_->mapping_progress = 0.0;
    
    emit mappingStateChanged(d_->mapping_state);
    emit mappingProgressChanged(d_->mapping_progress);
    emit mappingCancelled();
}

void MappingController::pauseMapping()
{
    if (!d_->is_mapping || d_->is_paused) {
        return;
    }
    
    if (d_->mapping_process) {
        d_->mapping_process->write("SIGSTOP\n");
    }
    
    d_->is_paused = true;
    d_->mapping_state = "已暂停";
    d_->monitor_timer->stop();
    
    emit mappingStateChanged(d_->mapping_state);
}

void MappingController::resumeMapping()
{
    if (!d_->is_paused) {
        return;
    }
    
    if (d_->mapping_process) {
        d_->mapping_process->write("SIGCONT\n");
    }
    
    d_->is_paused = false;
    d_->mapping_state = "建图中";
    d_->monitor_timer->start();
    
    emit mappingStateChanged(d_->mapping_state);
}

void MappingController::saveMap(const QString& filename)
{
    if (!d_->is_mapping) {
        emit error("未在建图状态,无法保存地图");
        return;
    }
    
    QString map_path = filename;
    if (!map_path.endsWith(".pgm")) {
        map_path += ".pgm";
    }
    
    // 检查磁盘空间
    if (!checkDiskSpace(QFileInfo(map_path).path())) {
        emit error("磁盘空间不足,无法保存地图");
        return;
    }
    
    // 调用map_server保存地图
    QProcess save_process;
    QString cmd = QString("rosrun map_server map_saver -f %1").arg(map_path);
    save_process.start(cmd);
    
    if (!save_process.waitForFinished(5000)) {
        emit error("保存地图超时");
        return;
    }
    
    if (save_process.exitCode() != 0) {
        emit error("保存地图失败: " + QString(save_process.readAllStandardError()));
        return;
    }
    
    d_->current_map_path = map_path;
    emit mapSaved(map_path);
}

void MappingController::loadMap(const QString& filename)
{
    if (d_->is_mapping) {
        emit error("正在建图中,无法加载地图");
        return;
    }
    
    if (!QFile::exists(filename)) {
        emit error("地图文件不存在");
        return;
    }
    
    // 启动map_server加载地图
    QProcess load_process;
    QString cmd = QString("rosrun map_server map_server %1").arg(filename);
    load_process.start(cmd);
    
    if (!load_process.waitForStarted()) {
        emit error("加载地图失败");
        return;
    }
    
    d_->current_map_path = filename;
    emit mapLoaded(filename);
}

bool MappingController::startAutoMapping(const QString& method, double resolution)
{
    if (d_->is_mapping || d_->is_auto_mapping) {
        return false;
    }
    
    d_->mapping_method = method;
    d_->resolution = resolution;
    
    // 启动自动建图
    startMapping();
    if (!d_->is_mapping) {
        return false;
    }
    
    d_->is_auto_mapping = true;
    emit autoMappingStarted();
    
    // 启动探索线程
    // TODO: 实现自动探索逻辑
    
    return true;
}

void MappingController::stopAutoMapping()
{
    if (!d_->is_auto_mapping) {
        return;
    }
    
    // 停止探索
    // TODO: 停止自动探索逻辑
    
    stopMapping();
    d_->is_auto_mapping = false;
    emit autoMappingStopped();
}

void MappingController::monitorMappingStatus()
{
    if (!d_->is_mapping || d_->is_paused) {
        return;
    }
    
    // 更新建图统计信息
    updateMappingStatistics();
    
    // 检查资源使用情况
    checkResourceUsage();
    
    // 检查建图质量
    checkMappingQuality();
    
    // 检查建图进度
    checkMappingProgress();
    
    // 发送诊断信息
    emitDiagnosticInfo();
}

void MappingController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    emit scanReceived(msg);
}

void MappingController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    emit mapReceived(msg);
    emit mapUpdated(msg);
}

bool MappingController::checkSensorsStatus()
{
    // TODO: 实现传感器状态检查
    return true;
}

bool MappingController::checkSystemResources()
{
    // 检查CPU和内存使用情况
    QProcess process;
    process.start("top -n 1 -b");
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    
    // 解析CPU和内存使用率
    // TODO: 实现资源使用率解析
    
    return true;
}

bool MappingController::checkDiskSpace(const QString& path)
{
    QStorageInfo storage(path);
    if (!storage.isValid() || !storage.isReady()) {
        return false;
    }
    
    // 要求至少100MB可用空间
    return storage.bytesAvailable() > 100 * 1024 * 1024;
}

void MappingController::handleMappingProcessOutput()
{
    if (!d_->mapping_process) {
        return;
    }
    
    QString output = d_->mapping_process->readAllStandardOutput();
    // 根据输出调整参数
    if (d_->mapping_method == "gmapping") {
        adjustGmappingParams(output);
    } else if (d_->mapping_method == "cartographer") {
        adjustCartographerParams(output);
    } else if (d_->mapping_method == "hector") {
        adjustHectorParams(output);
    }
}

void MappingController::handleMappingProcessError(QProcess::ProcessError error)
{
    QString error_msg;
    switch (error) {
        case QProcess::FailedToStart:
            error_msg = "建图进程启动失败";
            break;
        case QProcess::Crashed:
            error_msg = "建图进程崩溃";
            break;
        case QProcess::Timedout:
            error_msg = "建图进程超时";
            break;
        case QProcess::WriteError:
            error_msg = "向建图进程写入数据失败";
            break;
        case QProcess::ReadError:
            error_msg = "从建图进程读取数据失败";
            break;
        default:
            error_msg = "建图进程未知错误";
            break;
    }
    
    emit this->error(error_msg);
    stopMapping();
}

// Getter方法实现
bool MappingController::isMapping() const { return d_->is_mapping; }
bool MappingController::isAutoMapping() const { return d_->is_auto_mapping; }
bool MappingController::isPaused() const { return d_->is_paused; }
QString MappingController::mappingState() const { return d_->mapping_state; }
double MappingController::mappingProgress() const { return d_->mapping_progress; }
QString MappingController::mappingStatus() const { return d_->mapping_status; }
QString MappingController::getCurrentMapPath() const { return d_->current_map_path; }

// Setter方法实现
void MappingController::setMappingMethod(const QString& method) { d_->mapping_method = method; }
void MappingController::setUpdateRate(int rate) { d_->update_rate = rate; }
void MappingController::setMaxRange(double range) { d_->max_range = range; }
void MappingController::setMinRange(double range) { d_->min_range = range; }
void MappingController::setAngleMin(double angle) { d_->angle_min = angle; }
void MappingController::setAngleMax(double angle) { d_->angle_max = angle; }
void MappingController::setAngleIncrement(double increment) { d_->angle_increment = increment; }
void MappingController::setTimeIncrement(double increment) { d_->time_increment = increment; }
void MappingController::setScanTime(double time) { d_->scan_time = time; }
void MappingController::setIntensitiesEnabled(bool enabled) { d_->intensities_enabled = enabled; }

// 自动建图相关方法实现
void MappingController::setExplorationStrategy(const QString& strategy) { d_->exploration_strategy = strategy; }
void MappingController::setExplorationRadius(double radius) { d_->exploration_radius = radius; }
void MappingController::setMinFrontierSize(double size) { d_->min_frontier_size = size; }
void MappingController::setMaxExplorationTime(int minutes) { d_->max_exploration_time = minutes; }
void MappingController::setReturnHomeEnabled(bool enabled) { d_->return_home_enabled = enabled; }

// 私有辅助方法实现
void MappingController::updateMappingStatistics()
{
    // TODO: 实现建图统计信息更新
}

void MappingController::checkMappingQuality()
{
    // TODO: 实现建图质量检查
}

void MappingController::checkMappingProgress()
{
    // TODO: 实现建图进度检查
}

void MappingController::emitDiagnosticInfo()
{
    // TODO: 实现诊断信息发送
}

void MappingController::checkResourceUsage()
{
    // TODO: 实现资源使用情况检查
}

void MappingController::adjustGmappingParams(const QString& output)
{
    // TODO: 实现gmapping参数调整逻辑
}

void MappingController::adjustCartographerParams(const QString& output)
{
    // TODO: 实现cartographer参数调整逻辑
}

void MappingController::adjustHectorParams(const QString& output)
{
    // TODO: 实现hector参数调整逻辑
}

#include "mapping_controller.moc" 