/*
 * Copyright (c) 2025 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
    void moved(double x, double y);

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