#pragma once

#include <QDialog>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <memory>
#include "ros/robot_controller.h"

class QComboBox;
class QLabel;
class QDialogButtonBox;

class PlannerSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PlannerSettingsDialog(std::shared_ptr<RobotController> robot_controller, QWidget* parent = nullptr);
    ~PlannerSettingsDialog() override = default;

private slots:
    void onGlobalPlannerChanged(const QString& planner_name);
    void onLocalPlannerChanged(const QString& planner_name);
    void onAccepted();
    void onRejected();
    void updatePlannerDescriptions();

private:
    void setupUi();
    void connectSignalsAndSlots();
    QString getGlobalPlannerDescription(const QString& planner_name) const;
    QString getLocalPlannerDescription(const QString& planner_name) const;

    std::shared_ptr<RobotController> robot_controller_;
    
    // UI elements
    QComboBox* global_planner_combo_{nullptr};
    QComboBox* local_planner_combo_{nullptr};
    QLabel* global_planner_description_{nullptr};
    QLabel* local_planner_description_{nullptr};
    QPushButton* ok_button_{nullptr};
    QPushButton* cancel_button_{nullptr};
    QDialogButtonBox* button_box_{nullptr};
}; 