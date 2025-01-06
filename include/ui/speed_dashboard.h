#ifndef SPEED_DASHBOARD_H
#define SPEED_DASHBOARD_H

#include <QWidget>

class SpeedDashboard : public QWidget {
    Q_OBJECT

public:
    explicit SpeedDashboard(QWidget* parent = nullptr);

    void setLinearSpeed(double speed);
    void setAngularSpeed(double speed);
    
    double getLinearSpeed() const { return linear_speed_; }
    double getAngularSpeed() const { return angular_speed_; }

    static constexpr double MAX_LINEAR_SPEED = 2.0;  // m/s
    static constexpr double MAX_ANGULAR_SPEED = 1.0; // rad/s

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    double linear_speed_;
    double angular_speed_;
    
    void drawDashboard(QPainter& painter, const QRectF& rect, 
                      double value, double max_value, const QString& label);
};

#endif // SPEED_DASHBOARD_H 