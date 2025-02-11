#pragma once

#ifndef ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H
#define ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H

#include <QWidget>
#include <memory>

class SpeedDashboard : public QWidget
{
    Q_OBJECT

public:
    explicit SpeedDashboard(QWidget* parent = nullptr);
    ~SpeedDashboard() override;

    double linearSpeed() const;
    double angularSpeed() const;

public Q_SLOTS:
    void setLinearSpeed(double speed);
    void setAngularSpeed(double speed);
    void setMaxLinearSpeed(double speed);
    void setMaxAngularSpeed(double speed);

Q_SIGNALS:
    void linearSpeedChanged(double speed);
    void angularSpeedChanged(double speed);
    void maxLinearSpeedChanged(double speed);
    void maxAngularSpeedChanged(double speed);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void updateDisplay();
    void drawBackground(QPainter* painter);
    void drawScale(QPainter* painter);
    void drawNeedle(QPainter* painter);
    void drawLabels(QPainter* painter);

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif // ROBOT_CONTROL_GUI_SPEED_DASHBOARD_H 