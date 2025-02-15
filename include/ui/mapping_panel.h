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

#ifndef ROBOT_CONTROL_GUI_MAPPING_PANEL_H
#define ROBOT_CONTROL_GUI_MAPPING_PANEL_H

#include <QWidget>
#include <memory>
#include "ros/robot_controller.h"
#include "ui/rviz_view.h"
#include "ui/joystick_widget.h"

class QGroupBox;
class QComboBox;
class QPushButton;
class QLabel;
class QProgressBar;
class QSpinBox;
class QDoubleSpinBox;

class MappingPanel : public QWidget
{
    Q_OBJECT

public:
    explicit MappingPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent = nullptr);
    ~MappingPanel() override;
    void setRVizView(RVizView* view);

private slots:
    void onStartMapping();
    void onStopMapping();
    void onSaveMap();
    void onMappingMethodChanged(const QString& method);
    void onMappingProgressChanged(double progress);
    void onMappingStatusChanged(const QString& status);
    void updateMappingParameters();
    void onLinearJoystickMoved(double x, double y);
    void onAngularJoystickMoved(double x, double y);

private:
    void setupUI();
    void setupMappingGroup();
    void setupParametersGroup();
    void setupJoystickGroup();
    void setupRVizGroup();
    void connectSignals();
    void updateMethodDescription();
    QString getMappingMethodDescription(const QString& method) const;

    struct Private;
    std::unique_ptr<Private> d;
    RVizView* rviz_view_{nullptr};
    JoystickWidget* linear_joystick_{nullptr};
    JoystickWidget* angular_joystick_{nullptr};
    
    QGroupBox* mapping_group_{nullptr};
    QGroupBox* parameters_group_{nullptr};
    QGroupBox* joystick_group_{nullptr};
    QGroupBox* rviz_group_{nullptr};
};

#endif // ROBOT_CONTROL_GUI_MAPPING_PANEL_H 