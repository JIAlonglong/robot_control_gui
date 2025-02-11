#ifndef NAVIGATION_ACTION_BLOCK_H
#define NAVIGATION_ACTION_BLOCK_H

#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include "robot_controller.h"
#include "action_block.h"

class NavigationActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(double x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(double y READ y WRITE setY NOTIFY yChanged)
    Q_PROPERTY(double theta READ theta WRITE setTheta NOTIFY thetaChanged)
    Q_PROPERTY(double tolerance READ tolerance WRITE setTolerance NOTIFY toleranceChanged)

public:
    explicit NavigationActionBlock(std::shared_ptr<RobotController> robot_controller,
                                   QObject*                         parent = nullptr);
    ~NavigationActionBlock() override;

    bool isValid() const override
    {
        return d_->parameters["x"].toDouble() != 0.0 || 
               d_->parameters["y"].toDouble() != 0.0 || 
               d_->parameters["theta"].toDouble() != 0.0;
    }

    QString type() const override
    {
        return "导航动作";
    }
    QString name() const override
    {
        return tr("导航动作");
    }
    QString description() const override
    {
        return tr("控制机器人导航到指定位置");
    }
    QIcon icon() const override;
    bool  isRunning() const override
    {
        return d_->is_running;
    }
    void reset() override;

    double x() const
    {
        return d_->parameters["x"].toDouble();
    }
    void setX(double x)
    {
        setParameter("x", x);
        emit xChanged(x);
    }

    double y() const
    {
        return d_->parameters["y"].toDouble();
    }
    void setY(double y)
    {
        setParameter("y", y);
        emit yChanged(y);
    }

    double theta() const
    {
        return d_->parameters["theta"].toDouble();
    }
    void setTheta(double theta)
    {
        setParameter("theta", theta);
        emit thetaChanged(theta);
    }

    double tolerance() const
    {
        return d_->parameters["tolerance"].toDouble();
    }
    void setTolerance(double tolerance)
    {
        setParameter("tolerance", tolerance);
        emit toleranceChanged(tolerance);
    }

    QVariantMap properties() const override;
    void        setProperty(const QString& name, const QVariant& value) override;

    QJsonObject toJson() const override;
    void        fromJson(const QJsonObject& json) override;

public Q_SLOTS:
    void execute() override;
    void stop() override;
    void pause() override;
    void resume() override;

Q_SIGNALS:
    void xChanged(double x);
    void yChanged(double y);
    void thetaChanged(double theta);
    void toleranceChanged(double tolerance);
    void stateChanged(const QString& state);
    void failed(const QString& error);

private Q_SLOTS:
    void onNavigationCompleted(bool success);
    void onNavigationFeedback(const QString& status);

private:
    std::shared_ptr<RobotController> robot_controller_;
};

#endif  // NAVIGATION_ACTION_BLOCK_H