#pragma once

#ifndef PLANNER_SETTINGS_DIALOG_H
#define PLANNER_SETTINGS_DIALOG_H

#include <QComboBox>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <memory>
#include "robot_controller.h"

class QComboBox;
class QLabel;
class QDialogButtonBox;

class PlannerSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PlannerSettingsDialog(std::shared_ptr<RobotController> robot_controller,
                                   QWidget*                         parent = nullptr);
    ~PlannerSettingsDialog() override;

public Q_SLOTS:
    void onGlobalPlannerChanged(const QString& planner);
    void onLocalPlannerChanged(const QString& planner);
    void onPlannerFrequencyChanged(double frequency);
    void onGoalToleranceChanged(double tolerance);
    void onPathBiasChanged(double bias);
    void onRecoveryBehaviorChanged(bool enabled);
    void onClearingRotationChanged(bool allowed);
    void onAccepted();
    void onRejected();

private Q_SLOTS:
    void onApplyButtonClicked();
    void onResetButtonClicked();
    void onCloseButtonClicked();
    void updatePlannerDescriptions();

private:
    void    setupUi();
    void    connectSignalsAndSlots();
    QString getGlobalPlannerDescription(const QString& planner_name) const;
    QString getLocalPlannerDescription(const QString& planner_name) const;

    std::shared_ptr<RobotController> robot_controller_;

    // UI elements
    QComboBox*        global_planner_combo_{nullptr};
    QComboBox*        local_planner_combo_{nullptr};
    QLabel*           global_planner_description_{nullptr};
    QLabel*           local_planner_description_{nullptr};
    QPushButton*      ok_button_{nullptr};
    QPushButton*      cancel_button_{nullptr};
    QDialogButtonBox* button_box_{nullptr};

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif  // PLANNER_SETTINGS_DIALOG_H