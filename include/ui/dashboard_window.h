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

#ifndef DASHBOARD_WINDOW_H
#define DASHBOARD_WINDOW_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>

class DashboardWindow : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardWindow(QWidget* parent = nullptr);

    // 更新状态
    void updateSpeed(double linear_speed, double angular_speed);
    void updateBatteryLevel(double level);
    void updateStatus(const QString& status);

private:
    QLabel* speed_label_;
    QProgressBar* battery_bar_;
    QLabel* status_label_;
};

#endif // DASHBOARD_WINDOW_H 