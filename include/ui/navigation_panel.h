#pragma once

#include <QWidget>
#include <memory>
#include <map>
#include "ros/robot_controller.h"

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

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onSetInitialPose();
    void onAutoLocalization();
    void onSetGoal();
    void onCancelGoal();
    void onStartNavigation();
    void onPauseNavigation();
    void onStopNavigation();
    void onNavigationModeChanged(int index);
    void onNavigationStateChanged(RobotController::NavigationState state);
    void onLocalizationStateChanged(bool is_localized);
    void onLocalizationProgressChanged(double progress);
    void onNavigationProgressChanged(double progress);
    void onDistanceToGoalChanged(double distance);
    void onEstimatedTimeToGoalChanged(double time);
    void updateLocalizationStatus(const QString& status);
    void updateVelocityDisplay(double linear, double angular);
    void onEmergencyStop();
    void onJoystickMoved();
    void onPlannerSettings();

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

signals:
    void localizationStateChanged(bool is_localized);
    void localizationStatusChanged(const QString& status);
    void localizationProgressChanged(double progress);
}; 