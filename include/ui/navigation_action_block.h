#ifndef NAVIGATION_ACTION_BLOCK_H
#define NAVIGATION_ACTION_BLOCK_H

#include "ui/action_block.h"
#include "ros/robot_controller.h"
#include <geometry_msgs/PoseStamped.h>
#include <memory>

class NavigationActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(double x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(double y READ y WRITE setY NOTIFY yChanged)
    Q_PROPERTY(double theta READ theta WRITE setTheta NOTIFY thetaChanged)
    Q_PROPERTY(double tolerance READ tolerance WRITE setTolerance NOTIFY toleranceChanged)

public:
    explicit NavigationActionBlock(std::shared_ptr<RobotController> robot_controller, QObject* parent = nullptr);
    ~NavigationActionBlock() override;

    bool isValid() const override {
        return x_ != 0.0 || y_ != 0.0 || theta_ != 0.0;
    }

    QString type() const override { return "导航动作"; }
    QString name() const override { return tr("导航动作"); }
    QString description() const override { return tr("控制机器人导航到指定位置"); }
    QIcon icon() const override;
    bool isRunning() const override { return is_running_; }
    void reset() override;

    double x() const { return x_; }
    void setX(double x);

    double y() const { return y_; }
    void setY(double y);

    double theta() const { return theta_; }
    void setTheta(double theta);

    double tolerance() const { return tolerance_; }
    void setTolerance(double tolerance);

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
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    double tolerance_{0.1};
    bool is_running_{false};
};

#endif // NAVIGATION_ACTION_BLOCK_H 