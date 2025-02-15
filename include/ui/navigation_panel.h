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

#pragma once

#include <QWidget>
#include <memory>
#include <map>
#include "ros/robot_controller.h"
#include "ui/goal_setting_dialog.h"
#include "ui/initial_pose_dialog.h"
#include "ui/planner_settings_dialog.h"

class QLabel;
class QPushButton;
class QProgressBar;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QDial;
class RVizView;

class NavigationPanel : public QWidget {
    Q_OBJECT
public:
    explicit NavigationPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent = nullptr);
    ~NavigationPanel() override;

    void setRVizView(RVizView* view);

public slots:
    void onSetInitialPose();
    void onAutoLocalization();
    void onSetGoal();
    void onCancelGoal();
    void onStartNavigation();
    void onPauseNavigation();
    void onStopNavigation();
    void onEmergencyStop();
    void onPlannerSettings();
    void onNavigationModeChanged(int index);
    void onLocalizationStateChanged(const QString& state);
    void onLocalizationProgressChanged(double progress);
    void onNavigationStateChanged(const QString& state);
    void onNavigationProgressChanged(double progress);
    void onNavigationStatusChanged(const QString& status);
    void updateLocalizationStatus(const QString& status);
    void onDistanceToGoalChanged(double distance);
    void onEstimatedTimeToGoalChanged(double time);
    void updateVelocityDisplay(double linear, double angular);
    void onJoystickMoved();

signals:
    void localizationStateChanged(const QString& state);
    void localizationStatusChanged(const QString& status);
    void localizationProgressChanged(double progress);
    void startNavigationClicked();
    void stopNavigationClicked();
    void pauseNavigationClicked();
    void resumeNavigationClicked();

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    void setupUI();
    void setupJoystick();
    void setupKeyboardControl();
    void connectSignalsAndSlots();
    void updateLocalizationState(bool is_localized);
    void updateLocalizationProgress(double progress);
    void updateKeyboardVelocity();

    class NavigationPanelPrivate;
    std::unique_ptr<NavigationPanelPrivate> d_ptr;
    std::shared_ptr<RobotController> robot_controller_;
}; 