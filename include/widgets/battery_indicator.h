#ifndef ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H
#define ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H

#include <QWidget>
#include <QPainter>
#include <QColor>

class BatteryIndicator : public QWidget {
    Q_OBJECT
public:
    explicit BatteryIndicator(QWidget* parent = nullptr);
    ~BatteryIndicator() override;

    void setBatteryLevel(double level);
    double batteryLevel() const { return battery_level_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

private:
    QColor getBatteryColor() const;
    void drawBattery(QPainter& painter);
    void drawBatteryLevel(QPainter& painter);
    void drawBatteryText(QPainter& painter);

    double battery_level_{0.0};  // 0.0 - 100.0
    int border_width_{2};
    int terminal_width_{4};
    int terminal_height_{10};
    QColor border_color_{Qt::black};
    QColor background_color_{Qt::white};
};

#endif // ROBOT_CONTROL_GUI_BATTERY_INDICATOR_H 