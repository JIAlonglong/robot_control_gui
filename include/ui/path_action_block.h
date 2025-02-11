#pragma once

#ifndef PATH_ACTION_BLOCK_H
#define PATH_ACTION_BLOCK_H

#include "ui/action_block.h"
#include <memory>
#include <QList>
#include <QPointF>

class RobotController;

class PathActionBlock : public ActionBlock
{
    Q_OBJECT

public:
    explicit PathActionBlock(std::shared_ptr<RobotController> robot_controller, QObject* parent = nullptr);
    ~PathActionBlock() override;

    bool isValid() const override;
    QString type() const override { return "路径动作"; }
    QString name() const override { return tr("路径动作"); }
    QString description() const override { return tr("控制机器人沿指定路径运动"); }
    QIcon icon() const override;
    bool isRunning() const override { return is_running_; }
    void reset() override;

    QList<QPointF> waypoints() const;
    void setWaypoints(const QList<QPointF>& waypoints);
    void addWaypoint(const QPointF& point);
    void removeWaypoint(int index);
    void clearWaypoints();

    QVariantMap properties() const override;
    void setProperty(const QString& name, const QVariant& value) override;
    
    QJsonObject toJson() const override;
    void fromJson(const QJsonObject& json) override;

public Q_SLOTS:
    void execute() override;
    void stop() override;
    void pause() override;
    void resume() override;

Q_SIGNALS:
    void waypointsChanged();
    void failed(const QString& error);
    void stateChanged(const QString& state);

private Q_SLOTS:
    void onNavigationCompleted(bool success);
    void onNavigationFeedback(const QString& status);

private:
    void moveToNextWaypoint();

    std::shared_ptr<RobotController> robot_controller_;
    QList<QPointF> waypoints_;
    int current_waypoint_{-1};
    bool is_running_{false};
};

#endif // PATH_ACTION_BLOCK_H 