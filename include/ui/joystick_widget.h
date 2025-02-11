#ifndef ROBOT_CONTROL_GUI_JOYSTICK_WIDGET_H
#define ROBOT_CONTROL_GUI_JOYSTICK_WIDGET_H

#include <QWidget>
#include <QPoint>

class JoystickWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget* parent = nullptr);
    ~JoystickWidget() override;

    void reset();

signals:
    void linearJoystickMoved(double x, double y);
    void angularJoystickMoved(double x, double y);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void updateJoystickPosition();
    QPointF normalizePosition(const QPoint& pos) const;

    bool is_pressed_{false};
    QPoint stick_pos_;
    QPoint center_;
    int base_radius_{80};  // 底座圆的半径
    int stick_radius_{20}; // 摇杆圆的半径
};

#endif // ROBOT_CONTROL_GUI_JOYSTICK_WIDGET_H 