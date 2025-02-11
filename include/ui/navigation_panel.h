#pragma once

#include <QWidget>
#include <memory>
#include <map>
#include "ros/robot_controller.h"
#include "ui/goal_setting_dialog.h"

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

    void setRVizView(const std::shared_ptr<RVizView>& rviz_view);

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
    void setupUi();
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