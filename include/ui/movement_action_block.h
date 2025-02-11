#pragma once

#ifndef ROBOT_CONTROL_GUI_MOVEMENT_ACTION_BLOCK_H
#define ROBOT_CONTROL_GUI_MOVEMENT_ACTION_BLOCK_H

#include "ui/action_block.h"
#include <QIcon>
#include <memory>

class RobotController;

class MovementActionBlock : public ActionBlock {
    Q_OBJECT
    Q_PROPERTY(double linearVelocity READ linearVelocity WRITE setLinearVelocity NOTIFY linearVelocityChanged)
    Q_PROPERTY(double angularVelocity READ angularVelocity WRITE setAngularVelocity NOTIFY angularVelocityChanged)
    Q_PROPERTY(double duration READ duration WRITE setDuration NOTIFY durationChanged)

public:
    explicit MovementActionBlock(const std::shared_ptr<RobotController>& controller,
                               QObject* parent = nullptr);
    ~MovementActionBlock() override;

    bool isValid() const override {
        return d_->linear_velocity != 0.0 || d_->angular_velocity != 0.0;
    }

    QString type() const override { return "移动动作"; }
    QString name() const override { return tr("移动动作"); }
    QString description() const override { return tr("控制机器人按指定速度运动指定时间"); }
    QIcon icon() const override;
    bool isRunning() const override;

    // 属性访问
    double linearVelocity() const;
    double angularVelocity() const;
    double duration() const;

    // 属性设置
    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);
    void setDuration(double duration);

    // ActionBlock接口实现
    void execute() override;
    void stop() override;
    void pause() override;
    void resume() override;
    void reset() override;

    QVariantMap properties() const override;
    void setProperty(const QString& name, const QVariant& value) override;

Q_SIGNALS:
    void linearVelocityChanged(double velocity);
    void angularVelocityChanged(double velocity);
    void durationChanged(double duration);

private Q_SLOTS:
    void onTimeout();

private:
    struct Private;
    std::unique_ptr<Private> d_;
};

#endif // ROBOT_CONTROL_GUI_MOVEMENT_ACTION_BLOCK_H 