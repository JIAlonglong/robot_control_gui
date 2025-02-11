#ifndef ROBOT_STATUS_PANEL_H
#define ROBOT_STATUS_PANEL_H

#include <QLabel>
#include <QProgressBar>
#include <QWidget>

class RobotStatusPanel : public QWidget
{
    Q_OBJECT

public:
    explicit RobotStatusPanel(QWidget* parent = nullptr);

    // 更新状态
    void updateBatteryLevel(double level);
    void updateWifiStrength(int strength);
    void updateStatus(const QString& status);

    // 获取状态
    double getBatteryLevel() const;
    bool   isConnected() const;
    void   setConnected(bool connected);

private:
    QProgressBar* battery_bar_;
    QProgressBar* wifi_bar_;
    QLabel*       status_label_;
    QLabel*       battery_label_;
    QLabel*       wifi_label_;
    bool          is_connected_;
};

#endif  // ROBOT_STATUS_PANEL_H