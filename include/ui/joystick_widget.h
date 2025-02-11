#pragma once

#include <QWidget>
#include <QPointF>
#include <memory>

class JoystickWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget* parent = nullptr);
    ~JoystickWidget();

    void reset();
    QPointF normalizedPosition() const { return normalized_pos_; }
    double getLinearVelocity() const;
    double getAngularVelocity() const;

Q_SIGNALS:
    void linearJoystickMoved(double x, double y);
    void joystickMoved(double linear, double angular);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    struct Private;
    std::unique_ptr<Private> d_;
    QPointF normalizePosition(const QPoint& pos) const;

    bool is_pressed_{false};
    QPoint center_;
    QPoint stick_pos_;
    QPointF normalized_pos_;
    int base_radius_{50};
    int stick_radius_{10};
}; 