/**
 * @file speed_dashboard.h
 * @brief 速度仪表盘控件
 */

#ifndef SPEED_DASHBOARD_H
#define SPEED_DASHBOARD_H

#include <QWidget>

class SpeedDashboard : public QWidget {
    Q_OBJECT

public:
    explicit SpeedDashboard(QWidget* parent = nullptr);
    virtual ~SpeedDashboard() = default;
    
    void setLinearSpeed(double speed);
    void setAngularSpeed(double speed);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawDashboard(QPainter& painter, const QRectF& rect, 
                      double value, double maxValue,
                      const QString& label);

    double linear_speed_;
    double angular_speed_;
    static constexpr double MAX_LINEAR_SPEED = 2.0;  // m/s
    static constexpr double MAX_ANGULAR_SPEED = 1.0; // rad/s
};

#endif // SPEED_DASHBOARD_H 