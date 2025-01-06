/**
 * @file joystick_widget.h
 * @brief 虚拟摇杆控件
 */

#ifndef JOYSTICK_WIDGET_H
#define JOYSTICK_WIDGET_H

#include <QWidget>
#include <QPoint>
#include <QPointF>
#include <QTimer>

/**
 * @brief 虚拟摇杆控件类
 * 
 * 提供可视化的摇杆控制界面，用于控制机器人移动
 */
class JoystickWidget : public QWidget {
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget* parent = nullptr);
    virtual ~JoystickWidget() = default;

    /**
     * @brief 获取当前位置
     * @return QPointF 范围[-1,1]的坐标
     */
    QPointF getPosition() const {
        QPoint delta = stick_pos_ - center_pos_;
        double x = static_cast<double>(delta.x()) / radius_;
        double y = static_cast<double>(delta.y()) / radius_;
        return QPointF(x, y);
    }
    
    /**
     * @brief 设置当前位置
     * @param x 范围[-1,1]的X坐标
     * @param y 范围[-1,1]的Y坐标
     */
    void setPosition(double x, double y) {
        // 如果鼠标正在控制，则忽略键盘输入
        if (is_pressed_) return;
        
        // 确保输入在[-1,1]范围内
        x = qBound(-1.0, x, 1.0);
        y = qBound(-1.0, y, 1.0);
        
        // 计算像素坐标
        int px = static_cast<int>(x * radius_);
        int py = static_cast<int>(y * radius_);
        stick_pos_ = center_pos_ + QPoint(px, py);
        
        update();  // 触发重绘
        emit positionChanged(x, y);  // 发送位置改变信号
    }

signals:
    /**
     * @brief 摇杆位置变化信号
     * @param x 范围[-1,1]，代表左右移动
     * @param y 范围[-1,1]，代表前后移动
     */
    void positionChanged(double x, double y);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

private slots:
    void onUpdateTimer();

private:
    QPoint stick_pos_;      ///< 摇杆当前位置
    QPoint center_pos_;     ///< 摇杆中心位置
    int radius_;            ///< 摇杆活动范围半径
    bool is_pressed_;       ///< 摇杆是否被鼠标按下
    QTimer* update_timer_;  ///< 位置更新定时器
    
    void updateStickPosition(const QPoint& pos);
    void emitPosition();    ///< 发送当前位置信号
};

#endif // JOYSTICK_WIDGET_H 