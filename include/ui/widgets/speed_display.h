#ifndef ROBOT_CONTROL_GUI_SPEED_DISPLAY_H
#define ROBOT_CONTROL_GUI_SPEED_DISPLAY_H

#include <QColor>
#include <QPainter>
#include <QWidget>

class SpeedDisplay : public QWidget
{
    Q_OBJECT
public:
    explicit SpeedDisplay(QWidget* parent = nullptr);
    ~SpeedDisplay() override;

    void   updateSpeed(double linear, double angular);
    double linearSpeed() const
    {
        return linear_speed_;
    }
    double angularSpeed() const
    {
        return angular_speed_;
    }

protected:
    void  paintEvent(QPaintEvent* event) override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

private:
    void drawBackground(QPainter& painter);
    void drawSpeedGauge(QPainter& painter);
    void drawSpeedText(QPainter& painter);

    double linear_speed_{0.0};   // m/s
    double angular_speed_{0.0};  // rad/s
    double max_linear_speed_{2.0};
    double max_angular_speed_{2.0};

    int    border_width_{2};
    QColor border_color_{Qt::black};
    QColor background_color_{Qt::white};
    QColor gauge_color_{Qt::blue};
};

#endif  // ROBOT_CONTROL_GUI_SPEED_DISPLAY_H