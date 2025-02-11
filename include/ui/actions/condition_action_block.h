#pragma once

#ifndef ROBOT_CONTROL_GUI_CONDITION_ACTION_BLOCK_H
#define ROBOT_CONTROL_GUI_CONDITION_ACTION_BLOCK_H

#include <QJsonObject>
#include <QList>
#include <QString>
#include <QVariant>
#include <memory>
#include "action_block.h"

class RobotController;

/**
 * @class ConditionActionBlock
 * @brief 条件动作块类，用于根据条件执行不同的动作
 */
class ConditionActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(
        QString conditionType READ conditionType WRITE setConditionType NOTIFY conditionTypeChanged)
    Q_PROPERTY(double targetX READ targetX WRITE setTargetX NOTIFY targetXChanged)
    Q_PROPERTY(double targetY READ targetY WRITE setTargetY NOTIFY targetYChanged)
    Q_PROPERTY(double targetTheta READ targetTheta WRITE setTargetTheta NOTIFY targetThetaChanged)
    Q_PROPERTY(double positionTolerance READ positionTolerance WRITE setPositionTolerance NOTIFY
                   positionToleranceChanged)
    Q_PROPERTY(double angleTolerance READ angleTolerance WRITE setAngleTolerance NOTIFY
                   angleToleranceChanged)
    Q_PROPERTY(QString sensorName READ sensorName WRITE setSensorName NOTIFY sensorNameChanged)
    Q_PROPERTY(double sensorThreshold READ sensorThreshold WRITE setSensorThreshold NOTIFY
                   sensorThresholdChanged)
    Q_PROPERTY(ActionBlock* trueAction READ trueAction WRITE setTrueAction NOTIFY trueActionChanged)
    Q_PROPERTY(
        ActionBlock* falseAction READ falseAction WRITE setFalseAction NOTIFY falseActionChanged)

public:
    /**
     * @brief 条件类型枚举
     */
    enum class ConditionType {
        AtPosition,        ///< 到达指定位置
        AtOrientation,     ///< 到达指定朝向
        SensorThreshold,   ///< 传感器阈值
        ObstacleDetected,  ///< 检测到障碍物
        BatteryLow,        ///< 电池电量低
        Unknown            ///< 未知类型
    };
    Q_ENUM(ConditionType)

    /**
     * @brief 构造函数
     * @param robot_controller 机器人控制器
     * @param parent 父对象
     */
    explicit ConditionActionBlock(const std::shared_ptr<RobotController>& controller,
                                  QObject*                                parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~ConditionActionBlock() override;

    // ActionBlock接口实现
    QString type() const override
    {
        return "condition";
    }
    QString name() const override
    {
        return tr("条件动作");
    }
    QString description() const override
    {
        return tr("根据条件执行不同的动作");
    }
    QIcon icon() const override;
    bool  isRunning() const override
    {
        return d_->is_executing;
    }
    QVariantMap properties() const override;
    void        setProperty(const QString& name, const QVariant& value) override;
    QJsonObject toJson() const override;
    void        fromJson(const QJsonObject& json) override;

    // 属性访问
    QString conditionType() const
    {
        return d_->condition_type;
    }
    double targetX() const
    {
        return d_->target_x;
    }
    double targetY() const
    {
        return d_->target_y;
    }
    double targetTheta() const
    {
        return d_->target_theta;
    }
    double positionTolerance() const
    {
        return d_->position_tolerance;
    }
    double angleTolerance() const
    {
        return d_->angle_tolerance;
    }
    QString sensorName() const
    {
        return d_->sensor_name;
    }
    double sensorThreshold() const
    {
        return d_->sensor_threshold;
    }
    ActionBlock* trueAction() const
    {
        return d_->true_action;
    }
    ActionBlock* falseAction() const
    {
        return d_->false_action;
    }

    // 属性设置
    void setConditionType(const QString& type);
    void setTargetX(double x);
    void setTargetY(double y);
    void setTargetTheta(double theta);
    void setPositionTolerance(double tolerance);
    void setAngleTolerance(double tolerance);
    void setSensorName(const QString& name);
    void setSensorThreshold(double threshold);
    void setTrueAction(ActionBlock* action);
    void setFalseAction(ActionBlock* action);

    /**
     * @brief 验证参数是否有效
     * @return 如果参数有效返回true，否则返回false
     */
    bool validateParameters() const;

    bool isValid() const override
    {
        return d_->condition_type != "Unknown" &&
               (d_->true_action != nullptr || d_->false_action != nullptr);
    }

    void connectSignals();
    void start();
    void onActionStopped();

public Q_SLOTS:
    void execute() override;
    void stop() override;
    void pause() override;
    void resume() override;
    void reset() override;

Q_SIGNALS:
    void propertyChanged(const QString& name, const QVariant& value);
    void error(const QString& message);
    void started();
    void stopped();
    void completed(bool success);
    void runningChanged(bool running);
    void conditionTypeChanged(const QString& type);
    void targetXChanged(double x);
    void targetYChanged(double y);
    void targetThetaChanged(double theta);
    void positionToleranceChanged(double tolerance);
    void angleToleranceChanged(double tolerance);
    void sensorNameChanged(const QString& name);
    void sensorThresholdChanged(double threshold);
    void trueActionChanged(ActionBlock* action);
    void falseActionChanged(ActionBlock* action);
    void paused();
    void resumed();

private Q_SLOTS:
    void onActionStarted();
    void onActionCompleted(bool success);
    void onActionError(const QString& error);
    void onObstacleDetected(bool detected);
    void onBatteryLevelChanged(double level);

private:
    bool evaluateCondition() const;
    bool robotAtPosition(double x, double y, double tolerance) const;
    bool robotAtOrientation(double angle, double tolerance) const;
    bool sensorValue(const QString& name, double threshold) const;
    bool isObstacleDetected() const;
    bool isBatteryLow() const;

    struct Private {
        std::shared_ptr<RobotController> robot_controller;
        QString                          condition_type{"Unknown"};
        double                           target_x{0.0};
        double                           target_y{0.0};
        double                           target_theta{0.0};
        double                           position_tolerance{0.1};
        double                           angle_tolerance{0.1};
        QString                          sensor_name;
        double                           sensor_threshold{0.0};
        bool                             is_executing{false};
        bool                             is_paused{false};
        bool                             condition_met{false};
        ActionBlock*                     true_action{nullptr};
        ActionBlock*                     false_action{nullptr};
        ActionBlock*                     current_action{nullptr};
    };
    std::unique_ptr<Private> d_;
};

#endif  // ROBOT_CONTROL_GUI_CONDITION_ACTION_BLOCK_H