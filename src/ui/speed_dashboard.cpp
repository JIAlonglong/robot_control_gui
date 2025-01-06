/**
 * @file speed_dashboard.cpp
 * @brief 速度仪表盘控件实现
 */

#include "ui/speed_dashboard.h"
#include <QPainter>
#include <QtMath>

SpeedDashboard::SpeedDashboard(QWidget* parent)
    : QWidget(parent)
    , linear_speed_(0.0)    // 初始化线速度为0
    , angular_speed_(0.0)   // 初始化角速度为0
{
    // 设置最小尺寸以确保仪表盘有足够的显示空间
    setMinimumSize(200, 100);
}

// 设置线速度并触发重绘
void SpeedDashboard::setLinearSpeed(double speed)
{
    linear_speed_ = speed;
    update();  // 请求重绘界面
}

// 设置角速度并触发重绘
void SpeedDashboard::setAngularSpeed(double speed)
{
    angular_speed_ = speed;
    update();  // 请求重绘界面
}

// 绘制事件处理函数
void SpeedDashboard::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    // 启用抗锯齿，使绘制的图形更平滑
    painter.setRenderHint(QPainter::Antialiasing);

    // 将窗口水平分成两半，分别绘制线速度和角速度仪表盘
    int w = width() / 2;
    int h = height();
    QRectF leftRect(0, 0, w, h);          // 左侧线速度仪表盘区域
    QRectF rightRect(w, 0, w, h);         // 右侧角速度仪表盘区域

    // 分别绘制两个仪表盘
    drawDashboard(painter, leftRect, linear_speed_, MAX_LINEAR_SPEED, "Linear (m/s)");
    drawDashboard(painter, rightRect, angular_speed_, MAX_ANGULAR_SPEED, "Angular (rad/s)");
}

/**
 * @brief 绘制单个仪表盘
 * @param painter 绘图设备
 * @param rect 绘制区域
 * @param value 当前值
 * @param maxValue 最大值
 * @param label 仪表盘标签
 */
void SpeedDashboard::drawDashboard(QPainter& painter, const QRectF& rect, 
                                 double value, double maxValue,
                                 const QString& label)
{
    // 定义仪表盘的起始角度和跨度
    const int startAngle = 40;    // 起始角度（以3点钟方向为0度，顺时针为正）
    const int spanAngle = 280;    // 跨越角度
    
    // 绘制仪表盘外圈
    painter.setPen(QPen(QColor(200, 200, 200), 2));  // 设置灰色画笔
    painter.drawArc(rect.adjusted(10, 10, -10, -10), // 留出10像素边距
                   startAngle * 16, spanAngle * 16);  // Qt中角度需要乘以16

    // 绘制刻度线
    for (int i = 0; i <= 10; ++i) {
        // 计算每个刻度的角度
        double angle = startAngle + (spanAngle * i / 10.0);
        // 转换为弧度
        double radians = qDegreesToRadians(angle);
        
        // 计算刻度线的外端点和内端点
        QPointF outer(rect.center().x() + (rect.width() * 0.4 * qCos(radians)),
                     rect.center().y() - (rect.height() * 0.4 * qSin(radians)));
        QPointF inner(rect.center().x() + (rect.width() * 0.35 * qCos(radians)),
                     rect.center().y() - (rect.height() * 0.35 * qSin(radians)));
        
        // 绘制刻度线
        painter.drawLine(outer, inner);
    }

    // 绘制指针
    // 计算当前值对应的比例和角度
    double ratio = qBound(0.0, value / maxValue, 1.0);  // 确保比例在0-1之间
    double angle = startAngle + (spanAngle * ratio);
    double radians = qDegreesToRadians(angle);
    
    // 计算指针终点位置
    QPointF needle(rect.center().x() + (rect.width() * 0.38 * qCos(radians)),
                  rect.center().y() - (rect.height() * 0.38 * qSin(radians)));
    
    // 绘制指针（蓝色）
    painter.setPen(QPen(QColor(41, 128, 185), 2));
    painter.drawLine(rect.center(), needle);

    // 绘制标签和数值
    painter.setPen(Qt::black);
    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);
    
    // 在仪表盘下方绘制文本
    QRectF textRect = rect.adjusted(0, rect.height() * 0.7, 0, 0);
    painter.drawText(textRect, Qt::AlignCenter, 
                    QString("%1\n%2").arg(label).arg(value, 0, 'f', 2));
} 