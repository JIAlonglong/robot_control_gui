#ifndef JOYSTICK_WIDGET_H
#define JOYSTICK_WIDGET_H

#include <QWidget>
#include <QPoint>
#include <QTimer>

class JoystickWidget : public QWidget {
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget* parent = nullptr);
    void setPosition(double x, double y);

signals:
    void positionChanged(double x, double y);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void enterEvent(QEvent* event) override;

private:
    bool is_pressed_;
    int radius_;
    QPoint center_pos_;
    QPoint stick_pos_;
    QTimer* update_timer_;

    void emitPosition();
};

#endif // JOYSTICK_WIDGET_H 