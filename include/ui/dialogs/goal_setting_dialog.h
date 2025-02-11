#ifndef GOAL_SETTING_DIALOG_H
#define GOAL_SETTING_DIALOG_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

class GoalSettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GoalSettingDialog(const geometry_msgs::Pose& current_pose, QWidget* parent = nullptr);
    ~GoalSettingDialog() = default;

    geometry_msgs::PoseStamped getGoal() const;

private:
    QDoubleSpinBox*            x_spin_;
    QDoubleSpinBox*            y_spin_;
    QDoubleSpinBox*            yaw_spin_;
    geometry_msgs::PoseStamped goal_;

    void setupUI();
    void updateGoal();
};

#endif  // GOAL_SETTING_DIALOG_H