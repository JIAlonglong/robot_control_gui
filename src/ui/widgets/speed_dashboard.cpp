/**
 * Copyright (c) 2024 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file speed_dashboard.cpp
 * @brief 速度仪表盘组件的实现,用于以仪表盘形式显示机器人的速度信息
 * @author JIAlonglong
 */

#include "speed_dashboard.h"
#include <QPainter>
#include <QResizeEvent>
#include <QtMath>

struct SpeedDashboard::Private {
    double linear_speed_{0.0};
    double angular_speed_{0.0};
    double max_linear_speed_{1.0};
    double max_angular_speed_{1.0};
    int    margin_{10};
    QRectF linear_rect_;
    QRectF angular_rect_;
};

SpeedDashboard::SpeedDashboard(QWidget* parent) : QWidget(parent), d_(new Private)
{
    setMinimumSize(300, 150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

SpeedDashboard::~SpeedDashboard()
{
    delete d_;
}

double SpeedDashboard::linearSpeed() const
{
    return d_->linear_speed_;
}

double SpeedDashboard::angularSpeed() const
{
    return d_->angular_speed_;
}

void SpeedDashboard::setLinearSpeed(double speed)
{
    if (qFuzzyCompare(d_->linear_speed_, speed)) return;
    d_->linear_speed_ = speed;
    emit linearSpeedChanged(speed);
    update();
}

void SpeedDashboard::setAngularSpeed(double speed)
{
    if (qFuzzyCompare(d_->angular_speed_, speed)) return;
    d_->angular_speed_ = speed;
    emit angularSpeedChanged(speed);
    update();
}

void SpeedDashboard::setMaxLinearSpeed(double speed)
{
    if (qFuzzyCompare(d_->max_linear_speed_, speed)) return;
    d_->max_linear_speed_ = speed;
    emit maxLinearSpeedChanged(speed);
    update();
}

void SpeedDashboard::setMaxAngularSpeed(double speed)
{
    if (qFuzzyCompare(d_->max_angular_speed_, speed)) return;
    d_->max_angular_speed_ = speed;
    emit maxAngularSpeedChanged(speed);
    update();
}

void SpeedDashboard::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制背景
    drawBackground(&painter);

    // 绘制刻度
    drawScale(&painter);

    // 绘制指针
    drawNeedle(&painter);

    // 绘制标签
    drawLabels(&painter);
}

void SpeedDashboard::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    updateDisplay();
}

void SpeedDashboard::updateDisplay()
{
    int w = width() - 2 * d_->margin_;
    int h = (height() - 3 * d_->margin_) / 2;

    d_->linear_rect_  = QRectF(d_->margin_, d_->margin_, w, h);
    d_->angular_rect_ = QRectF(d_->margin_, h + 2 * d_->margin_, w, h);
}

void SpeedDashboard::drawBackground(QPainter* painter)
{
    painter->save();

    // 线速度仪表盘背景
    painter->setBrush(Qt::white);
    painter->setPen(QPen(Qt::black, 2));
    painter->drawEllipse(d_->linear_rect_);

    // 角速度仪表盘背景
    painter->drawEllipse(d_->angular_rect_);

    painter->restore();
}

void SpeedDashboard::drawScale(QPainter* painter)
{
    painter->save();

    // 绘制线速度刻度
    drawSpeedScale(painter, d_->linear_rect_, d_->max_linear_speed_);

    // 绘制角速度刻度
    drawSpeedScale(painter, d_->angular_rect_, d_->max_angular_speed_);

    painter->restore();
}

void SpeedDashboard::drawSpeedScale(QPainter* painter, const QRectF& rect, double max_speed)
{
    const int    num_major_ticks = 5;
    const int    num_minor_ticks = 4;
    const double start_angle     = 225;
    const double span_angle      = 270;

    painter->save();

    QPointF center = rect.center();
    double  radius = rect.width() * 0.4;

    // 绘制主刻度
    for (int i = 0; i <= num_major_ticks; ++i) {
        double  angle = start_angle + span_angle * i / num_major_ticks;
        double  rad   = qDegreesToRadians(angle);
        QPointF outer(center.x() + radius * qCos(rad), center.y() - radius * qSin(rad));
        QPointF inner(center.x() + radius * 0.9 * qCos(rad), center.y() - radius * 0.9 * qSin(rad));

        painter->setPen(QPen(Qt::black, 2));
        painter->drawLine(outer, inner);

        // 绘制刻度值
        double       value = max_speed * i / num_major_ticks;
        QString      text  = QString::number(value, 'f', 1);
        QFontMetrics fm(painter->font());
        QRectF       text_rect(outer.x() - 20, outer.y() - 10, 40, 20);
        painter->drawText(text_rect, Qt::AlignCenter, text);
    }

    // 绘制次刻度
    painter->setPen(QPen(Qt::black, 1));
    for (int i = 0; i < num_major_ticks * num_minor_ticks; ++i) {
        double  angle = start_angle + span_angle * i / (num_major_ticks * num_minor_ticks);
        double  rad   = qDegreesToRadians(angle);
        QPointF outer(center.x() + radius * qCos(rad), center.y() - radius * qSin(rad));
        QPointF inner(center.x() + radius * 0.95 * qCos(rad),
                      center.y() - radius * 0.95 * qSin(rad));
        painter->drawLine(outer, inner);
    }

    painter->restore();
}

void SpeedDashboard::drawNeedle(QPainter* painter)
{
    painter->save();

    // 绘制线速度指针
    drawSpeedNeedle(painter, d_->linear_rect_, d_->linear_speed_, d_->max_linear_speed_);

    // 绘制角速度指针
    drawSpeedNeedle(painter, d_->angular_rect_, d_->angular_speed_, d_->max_angular_speed_);

    painter->restore();
}

void SpeedDashboard::drawSpeedNeedle(QPainter* painter, const QRectF& rect, double speed,
                                     double max_speed)
{
    const double start_angle = 225;
    const double span_angle  = 270;

    QPointF center = rect.center();
    double  radius = rect.width() * 0.4;

    // 计算指针角度
    double ratio = qBound(0.0, speed / max_speed, 1.0);
    double angle = start_angle + span_angle * ratio;
    double rad   = qDegreesToRadians(angle);

    // 绘制指针
    QPointF needle_tip(center.x() + radius * qCos(rad), center.y() - radius * qSin(rad));

    painter->setPen(QPen(Qt::red, 2));
    painter->drawLine(center, needle_tip);

    // 绘制中心点
    painter->setBrush(Qt::red);
    painter->setPen(Qt::NoPen);
    painter->drawEllipse(center, 5, 5);
}

void SpeedDashboard::drawLabels(QPainter* painter)
{
    painter->save();

    QFont font = painter->font();
    font.setPointSize(10);
    painter->setFont(font);

    // 绘制线速度标签
    QString linear_text = QString("线速度: %1 m/s").arg(d_->linear_speed_, 0, 'f', 2);
    painter->drawText(d_->linear_rect_, Qt::AlignBottom | Qt::AlignHCenter, linear_text);

    // 绘制角速度标签
    QString angular_text = QString("角速度: %1 rad/s").arg(d_->angular_speed_, 0, 'f', 2);
    painter->drawText(d_->angular_rect_, Qt::AlignBottom | Qt::AlignHCenter, angular_text);

    painter->restore();
}