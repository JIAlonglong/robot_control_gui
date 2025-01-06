#ifndef ROBOT_STATUS_PANEL_H
#define ROBOT_STATUS_PANEL_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>

class RobotStatusPanel : public QWidget {
    Q_OBJECT

public:
    explicit RobotStatusPanel(QWidget* parent = nullptr);

    void setBatteryLevel(double level);
    double getBatteryLevel() const { return battery_level_; }
    
    void setConnected(bool connected);
    bool isConnected() const { return is_connected_; }
    
    void updateVelocity(double linear, double angular);
    void updateBatteryLevel(int percentage);
    void updateWorkMode(const QString& mode);

private:
    QLabel* battery_label_;
    QLabel* connection_label_;
    QLabel* velocity_label_;
    QProgressBar* battery_progress_;
    QLabel* mode_label_;
    
    double battery_level_;
    bool is_connected_;
    
    void updateBatteryDisplay();
    void updateConnectionDisplay();
    void setupUI();
};

#endif // ROBOT_STATUS_PANEL_H 