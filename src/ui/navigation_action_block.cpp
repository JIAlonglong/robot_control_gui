#include "ui/navigation_action_block.h"
#include "ros/robot_controller.h"
#include <QDebug>
#include <cmath>
#include <QIcon>

struct NavigationActionBlock::Private {
    std::shared_ptr<RobotController> robot_controller;
    double target_x{0.0};
    double target_y{0.0};
    double target_theta{0.0};
    double tolerance{0.1};
    bool is_running{false};
};

NavigationActionBlock::NavigationActionBlock(std::shared_ptr<RobotController> robot_controller, QObject* parent)
    : ActionBlock(parent)
    , robot_controller_(robot_controller)
{
    connect(robot_controller_.get(), &RobotController::navigationCompleted,
            this, &NavigationActionBlock::onNavigationCompleted);
    connect(robot_controller_.get(), &RobotController::navigationFeedback,
            this, &NavigationActionBlock::onNavigationFeedback);
}

NavigationActionBlock::~NavigationActionBlock() = default;

QIcon NavigationActionBlock::icon() const
{
    return QIcon(":/icons/navigation.png");
}

bool NavigationActionBlock::isRunning() const
{
    return d_->is_running;
}

bool NavigationActionBlock::isValid() const
{
    return d_->robot_controller != nullptr;
}

double NavigationActionBlock::targetX() const
{
    return d_->target_x;
}

double NavigationActionBlock::targetY() const
{
    return d_->target_y;
}

double NavigationActionBlock::targetTheta() const
{
    return d_->target_theta;
}

double NavigationActionBlock::tolerance() const
{
    return d_->tolerance;
}

void NavigationActionBlock::setTargetX(double x)
{
    if (qFuzzyCompare(d_->target_x, x)) {
        return;
    }
    d_->target_x = x;
    Q_EMIT targetXChanged(x);
    Q_EMIT propertyChanged("targetX", x);
}

void NavigationActionBlock::setTargetY(double y)
{
    if (qFuzzyCompare(d_->target_y, y)) {
        return;
    }
    d_->target_y = y;
    Q_EMIT targetYChanged(y);
    Q_EMIT propertyChanged("targetY", y);
}

void NavigationActionBlock::setTargetTheta(double theta)
{
    if (qFuzzyCompare(d_->target_theta, theta)) {
        return;
    }
    d_->target_theta = theta;
    Q_EMIT targetThetaChanged(theta);
    Q_EMIT propertyChanged("targetTheta", theta);
}

void NavigationActionBlock::setTolerance(double tolerance)
{
    if (qFuzzyCompare(d_->tolerance, tolerance)) {
        return;
    }
    d_->tolerance = tolerance;
    Q_EMIT toleranceChanged(tolerance);
    Q_EMIT propertyChanged("tolerance", tolerance);
}

QVariantMap NavigationActionBlock::properties() const
{
    return {
        {"targetX", d_->target_x},
        {"targetY", d_->target_y},
        {"targetTheta", d_->target_theta},
        {"tolerance", d_->tolerance}
    };
}

void NavigationActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "targetX") {
        setTargetX(value.toDouble());
    } else if (name == "targetY") {
        setTargetY(value.toDouble());
    } else if (name == "targetTheta") {
        setTargetTheta(value.toDouble());
    } else if (name == "tolerance") {
        setTolerance(value.toDouble());
    }
}

QJsonObject NavigationActionBlock::toJson() const
{
    return QJsonObject{
        {"type", type()},
        {"targetX", d_->target_x},
        {"targetY", d_->target_y},
        {"targetTheta", d_->target_theta},
        {"tolerance", d_->tolerance}
    };
}

void NavigationActionBlock::fromJson(const QJsonObject& json)
{
    setTargetX(json["targetX"].toDouble());
    setTargetY(json["targetY"].toDouble());
    setTargetTheta(json["targetTheta"].toDouble());
    setTolerance(json["tolerance"].toDouble());
}

void NavigationActionBlock::execute()
{
    if (!robot_controller_) {
        Q_EMIT failed(tr("未连接到机器人"));
        return;
    }

    d_->is_running = true;
    Q_EMIT started();
    robot_controller_->navigateTo(d_->target_x, d_->target_y, d_->target_theta, d_->tolerance);
}

void NavigationActionBlock::stop()
{
    if (!robot_controller_ || !d_->is_running) {
        return;
    }

    robot_controller_->stopNavigation();
    d_->is_running = false;
    Q_EMIT stopped();
}

void NavigationActionBlock::pause()
{
    if (!robot_controller_ || !d_->is_running) {
        return;
    }

    robot_controller_->pauseNavigation();
    Q_EMIT paused();
}

void NavigationActionBlock::resume()
{
    if (!robot_controller_ || !d_->is_running) {
        return;
    }

    robot_controller_->resumeNavigation();
    Q_EMIT resumed();
}

void NavigationActionBlock::reset()
{
    stop();
    setTargetX(0.0);
    setTargetY(0.0);
    setTargetTheta(0.0);
    setTolerance(0.1);
}

void NavigationActionBlock::onNavigationCompleted(bool success)
{
    if (!d_->is_running) {
        return;
    }

    d_->is_running = false;
    if (success) {
        Q_EMIT completed(true);
    } else {
        Q_EMIT failed(tr("导航失败"));
    }
}

void NavigationActionBlock::onNavigationFeedback(const QString& status)
{
    Q_EMIT stateChanged(status);
} 