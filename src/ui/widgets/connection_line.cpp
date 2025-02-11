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
 * @file connection_line.cpp
 * @brief 连接线组件的实现,用于在动作编辑器中显示动作块之间的连接关系
 * @author JIAlonglong
 */

#include "connection_line.h"
#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionGraphicsItem>
#include <QtMath>

ConnectionLine::ConnectionLine(ActionBlockWidget* from, ActionBlockWidget* to,
                               QGraphicsItem* parent)
    : QGraphicsPathItem(parent), from_(from), to_(to)
{
    setPen(QPen(QColor(80, 80, 80), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    setZValue(-1);  // 确保线条在块的下方
    updatePosition();
}

void ConnectionLine::updatePosition()
{
    if (!from_ || !to_) {
        return;
    }

    QPointF startPos = from_->pos();
    QPointF endPos   = to_->pos();

    // 创建贝塞尔曲线的控制点
    qreal   dx        = endPos.x() - startPos.x();
    qreal   minOffset = 50.0;                             // 最小控制点偏移
    qreal   offset    = qMax(minOffset, qAbs(dx) * 0.5);  // 动态调整控制点偏移
    QPointF c1(startPos.x() + offset, startPos.y());
    QPointF c2(endPos.x() - offset, endPos.y());

    // 设置路径
    QPainterPath path;
    path.moveTo(startPos);
    path.cubicTo(c1, c2, endPos);
    setPath(path);
}

void ConnectionLine::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                           QWidget* widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    if (!from_ || !to_) {
        return;
    }

    // 启用抗锯齿
    painter->setRenderHint(QPainter::Antialiasing);

    // 创建线条渐变
    QLinearGradient gradient(path().pointAtPercent(0), path().pointAtPercent(1));
    QColor          startColor(80, 80, 80);
    QColor          endColor(120, 120, 120);
    gradient.setColorAt(0, startColor);
    gradient.setColorAt(0.5, endColor);
    gradient.setColorAt(1, startColor);

    // 绘制路径阴影
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(0, 0, 0, 30));
    painter->translate(2, 2);
    painter->drawPath(path());
    painter->translate(-2, -2);

    // 绘制主路径
    QPen mainPen(gradient, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(mainPen);
    painter->drawPath(path());

    // 绘制箭头
    QPointF endPos    = to_->pos();
    QPointF direction = path().pointAtPercent(0.95) - endPos;
    direction         = direction / qSqrt(QPointF::dotProduct(direction, direction));

    // 创建箭头渐变
    QRadialGradient arrowGradient(endPos, 12);
    arrowGradient.setColorAt(0, endColor);
    arrowGradient.setColorAt(1, startColor);

    // 绘制箭头
    painter->setPen(Qt::NoPen);
    painter->setBrush(arrowGradient);
    QPointF arrowP1 =
        endPos + QPointF(direction.x() - direction.y(), direction.y() + direction.x()) * 12;
    QPointF arrowP2 =
        endPos + QPointF(direction.x() + direction.y(), direction.y() - direction.x()) * 12;

    QPainterPath arrowPath;
    arrowPath.moveTo(endPos);
    arrowPath.lineTo(arrowP1);
    arrowPath.lineTo(arrowP2);
    arrowPath.lineTo(endPos);

    // 绘制箭头阴影
    painter->setBrush(QColor(0, 0, 0, 30));
    painter->translate(2, 2);
    painter->drawPath(arrowPath);
    painter->translate(-2, -2);

    // 绘制箭头主体
    painter->setBrush(arrowGradient);
    painter->drawPath(arrowPath);
}

#include "connection_line.moc"