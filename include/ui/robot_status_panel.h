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

#ifndef ROBOT_STATUS_PANEL_H
#define ROBOT_STATUS_PANEL_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>

class RobotStatusPanel : public QWidget {
    Q_OBJECT

public:
    explicit RobotStatusPanel(QWidget* parent = nullptr);

    // 更新状态
    void updateBatteryLevel(double level);
    void updateWifiStrength(int strength);
    void updateStatus(const QString& status);

    // 获取状态
    double getBatteryLevel() const;
    bool isConnected() const;
    void setConnected(bool connected);

private:
    QProgressBar* battery_bar_;
    QProgressBar* wifi_bar_;
    QLabel* status_label_;
    QLabel* battery_label_;
    QLabel* wifi_label_;
    bool is_connected_;
};

#endif // ROBOT_STATUS_PANEL_H 