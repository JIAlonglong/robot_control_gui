#include "ui/speed_display.h"
#include <QPainter>
#include <QVBoxLayout>
#include <QLabel>

struct SpeedDisplay::Private {
    double linear_velocity_{0.0};
    double angular_velocity_{0.0};
};

SpeedDisplay::SpeedDisplay(QWidget* parent)
    : QWidget(parent)
    , d_(new Private)
{
    setMinimumSize(200, 100);
}

SpeedDisplay::~SpeedDisplay()
{
    delete d_;
}

void SpeedDisplay::setLinearVelocity(double velocity)
{
    d_->linear_velocity_ = velocity;
    update();
}

void SpeedDisplay::setAngularVelocity(double velocity)
{
    d_->angular_velocity_ = velocity;
    update();
}

void SpeedDisplay::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制背景
    painter.fillRect(rect(), Qt::white);
    painter.setPen(Qt::black);
    painter.drawRect(rect().adjusted(0, 0, -1, -1));
    
    // 设置字体
    QFont font = painter.font();
    font.setPointSize(12);
    painter.setFont(font);
    
    // 显示速度值
    QString linear_text = QString("线速度: %1 m/s").arg(d_->linear_velocity_, 0, 'f', 2);
    QString angular_text = QString("角速度: %1 rad/s").arg(d_->angular_velocity_, 0, 'f', 2);
    
    QRectF linear_rect = rect().adjusted(10, 10, -10, -rect().height()/2);
    QRectF angular_rect = rect().adjusted(10, rect().height()/2, -10, -10);
    
    painter.drawText(linear_rect, Qt::AlignLeft | Qt::AlignVCenter, linear_text);
    painter.drawText(angular_rect, Qt::AlignLeft | Qt::AlignVCenter, angular_text);
} 