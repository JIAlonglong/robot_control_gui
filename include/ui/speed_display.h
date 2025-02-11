#pragma once

#include <QWidget>
#include <memory>

class SpeedDisplay : public QWidget {
    Q_OBJECT

public:
    explicit SpeedDisplay(QWidget* parent = nullptr);
    ~SpeedDisplay();

    void setLinearVelocity(double velocity);
    void setAngularVelocity(double velocity);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct Private;
    std::unique_ptr<Private> d_;
}; 