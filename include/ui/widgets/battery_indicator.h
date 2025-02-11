/**
 * Copyright (c) 2024 JIAlonglong
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
 * 
 * @file battery_indicator.h
 * @brief 电池指示器组件的头文件,定义了用于显示机器人电池状态的组件
 * @author JIAlonglong
 */

#ifndef ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H
#define ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H

#include <QColor>
#include <QPainter>
#include <QWidget>

class BatteryIndicator : public QWidget
{
    Q_OBJECT
public:
    explicit BatteryIndicator(QWidget* parent = nullptr);
    ~BatteryIndicator() override;

    void   setBatteryLevel(double level);
    double batteryLevel() const
    {
        return battery_level_;
    }

protected:
    void  paintEvent(QPaintEvent* event) override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

private:
    QColor getBatteryColor() const;
    void   drawBattery(QPainter& painter);
    void   drawBatteryLevel(QPainter& painter);
    void   drawBatteryText(QPainter& painter);

    double battery_level_{0.0};  // 0.0 - 100.0
    int    border_width_{2};
    int    terminal_width_{4};
    int    terminal_height_{10};
    QColor border_color_{Qt::black};
    QColor background_color_{Qt::white};
};

#endif  // ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H