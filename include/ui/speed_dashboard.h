#ifndef ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H
#define ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H

#include <QWidget>
#include <memory>

class SpeedDashboard : public QWidget
{
    Q_OBJECT

public:
    explicit SpeedDashboard(QWidget* parent = nullptr);
    ~SpeedDashboard() override;

public slots:
    void setLinearSpeed(double speed);
    void setAngularSpeed(double speed);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void setupUi();

    double current_linear_speed_{0.0};
    double current_angular_speed_{0.0};
    const double max_linear_speed_{1.0};   // m/s
    const double max_angular_speed_{1.0};  // rad/s
};

#endif // ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H 