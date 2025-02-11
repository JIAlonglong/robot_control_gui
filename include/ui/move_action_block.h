#pragma once

#ifndef ROBOT_CONTROL_GUI_MOVE_ACTION_BLOCK_H
#define ROBOT_CONTROL_GUI_MOVE_ACTION_BLOCK_H

#include "action_block.h"
#include <memory>

class RobotController;

class MoveActionBlock : public ActionBlock {
    Q_OBJECT

public:
    explicit MoveActionBlock(const std::shared_ptr<RobotController>& controller, QObject* parent = nullptr);
    ~MoveActionBlock() override;

    QString type() const override { return type_; }
    QString name() const override { return name_; }
    QString description() const override { return description_; }
    QIcon icon() const override;

    bool isRunning() const override { return is_running_; }
    QVariantMap properties() const override;
    void setProperty(const QString& name, const QVariant& value) override;

    void execute() override;
    void stop() override;
    void reset() override;
    void pause() override;
    void resume() override;

    bool isValid() const override;

private:
    void onNavigationCompleted(bool success);
    void onNavigationFailed(const QString& error);

    std::shared_ptr<RobotController> robot_controller_;
    bool is_running_{false};
    double target_x_{0.0};
    double target_y_{0.0};
    double target_theta_{0.0};
    double tolerance_{0.1};
};

#endif // ROBOT_CONTROL_GUI_MOVE_ACTION_BLOCK_H 