/**
 * @file robot_status_panel.h
 * @brief 机器人状态显示面板
 */

#ifndef ROBOT_STATUS_PANEL_H
#define ROBOT_STATUS_PANEL_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>

/**
 * @brief 机器人状态显示面板类
 * 
 * 显示机器人的关键状态信息，如电量、速度、工作模式等
 */
class RobotStatusPanel : public QWidget {
    Q_OBJECT

public:
    explicit RobotStatusPanel(QWidget* parent = nullptr);

public slots:
    /**
     * @brief 更新电池电量
     * @param percentage 电量百分比(0-100)
     */
    void updateBatteryLevel(int percentage);

    /**
     * @brief 更新机器人速度
     * @param linear_x 线速度(m/s)
     * @param angular_z 角速度(rad/s)
     */
    void updateVelocity(double linear_x, double angular_z);

    /**
     * @brief 更新工作模式
     * @param mode 当前工作模式
     */
    void updateWorkMode(const QString& mode);

private:
    QLabel* battery_label_;
    QProgressBar* battery_progress_;
    QLabel* velocity_label_;
    QLabel* mode_label_;

    void setupUI();
};

#endif // ROBOT_STATUS_PANEL_H 