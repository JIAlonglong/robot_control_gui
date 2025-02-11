#ifndef DASHBOARD_WINDOW_H
#define DASHBOARD_WINDOW_H

#include <QLabel>
#include <QProgressBar>
#include <QWidget>

class DashboardWindow : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardWindow(QWidget* parent = nullptr);

    // 更新状态
    void updateSpeed(double linear_speed, double angular_speed);
    void updateBatteryLevel(double level);
    void updateStatus(const QString& status);

private:
    QLabel*       speed_label_;
    QProgressBar* battery_bar_;
    QLabel*       status_label_;
};

#endif  // DASHBOARD_WINDOW_H