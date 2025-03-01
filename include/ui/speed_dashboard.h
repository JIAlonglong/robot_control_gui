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