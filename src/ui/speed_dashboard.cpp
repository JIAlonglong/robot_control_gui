/**
 * @file speed_dashboard.cpp
 * @brief 速度仪表盘控件实现
 */

#include "ui/speed_dashboard.h"
#include <QPainter>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

SpeedDashboard::SpeedDashboard(QWidget* parent)
    : QWidget(parent)
{
    setupUi();
    setMinimumSize(200, 200);
}

SpeedDashboard::~SpeedDashboard() = default;

void SpeedDashboard::setupUi()
{
    setStyleSheet("background-color: transparent;");
}

void SpeedDashboard::setLinearSpeed(double speed)
{
    current_linear_speed_ = speed;
    update();  // 触发重绘
}

void SpeedDashboard::setAngularSpeed(double speed)
{
    current_angular_speed_ = speed;
    update();  // 触发重绘
}

void SpeedDashboard::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 计算中心点和半径
    QPointF center(width() / 2.0, height() / 2.0);
    double radius = qMin(width(), height()) / 2.0 - 20;
    
    // 绘制外圈
    painter.setPen(QPen(QColor(200, 200, 200), 2));
    painter.drawEllipse(center, radius, radius);
    
    // 绘制刻度线和数值
    painter.save();
    painter.translate(center);
    for (int i = 0; i < 360; i += 30) {
        painter.rotate(30);
        if (i % 90 == 0) {
            painter.setPen(QPen(QColor(100, 100, 100), 2));
            painter.drawLine(QPointF(radius - 15, 0), QPointF(radius, 0));
            
            // 绘制数值
            painter.save();
            painter.rotate(-i - 90);  // 修正文本方向
            double value = (i - 90) / 180.0;  // 将角度转换为-1到1的范围
            QString text = QString::number(value, 'f', 1);
            QFontMetrics fm(painter.font());
            int text_width = fm.horizontalAdvance(text);
            painter.drawText(QPointF(-text_width/2, -radius + 12), text);
            painter.restore();
        } else {
            painter.setPen(QPen(QColor(200, 200, 200), 1));
            painter.drawLine(QPointF(radius - 10, 0), QPointF(radius, 0));
        }
    }
    painter.restore();
    
    // 绘制线速度指针（蓝色）
    double linear_angle = -90 + (current_linear_speed_ / max_linear_speed_) * 180;
    painter.save();
    painter.translate(center);
    painter.rotate(linear_angle);
    
    // 绘制指针阴影
    painter.setPen(QPen(QColor(0, 0, 0, 30), 4));
    painter.drawLine(QPointF(2, 2), QPointF(radius - 28, 2));
    
    // 绘制指针
    QLinearGradient linearGradient(0, 0, radius, 0);
    linearGradient.setColorAt(0, QColor(51, 154, 255));
    linearGradient.setColorAt(1, QColor(0, 119, 255));
    painter.setPen(QPen(linearGradient, 3));
    painter.drawLine(QPointF(0, 0), QPointF(radius - 30, 0));
    painter.restore();
    
    // 绘制角速度指针（橙色）
    double angular_angle = -90 + (current_angular_speed_ / max_angular_speed_) * 180;
    painter.save();
    painter.translate(center);
    painter.rotate(angular_angle);
    
    // 绘制指针阴影
    painter.setPen(QPen(QColor(0, 0, 0, 30), 4));
    painter.drawLine(QPointF(2, 2), QPointF(radius - 48, 2));
    
    // 绘制指针
    QLinearGradient angularGradient(0, 0, radius, 0);
    angularGradient.setColorAt(0, QColor(255, 87, 34));
    angularGradient.setColorAt(1, QColor(244, 67, 54));
    painter.setPen(QPen(angularGradient, 3));
    painter.drawLine(QPointF(0, 0), QPointF(radius - 50, 0));
    painter.restore();
    
    // 绘制中心装饰
    // 外圈
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(240, 240, 240));
    painter.drawEllipse(center, 15, 15);
    // 内圈
    painter.setBrush(QColor(51, 51, 51));
    painter.drawEllipse(center, 5, 5);
    
    // 绘制图例
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);
    
    // 线速度图例
    int legend_y = height() - 30;
    painter.setPen(QColor(51, 154, 255));
    painter.drawLine(10, legend_y, 30, legend_y);
    painter.setPen(QColor(51, 51, 51));
    painter.drawText(35, legend_y + 5, tr("线速度"));
    
    // 角速度图例
    painter.setPen(QColor(255, 87, 34));
    painter.drawLine(90, legend_y, 110, legend_y);
    painter.setPen(QColor(51, 51, 51));
    painter.drawText(115, legend_y + 5, tr("角速度"));
} 