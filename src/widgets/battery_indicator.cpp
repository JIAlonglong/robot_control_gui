#include "widgets/battery_indicator.h"
#include <QPainter>
#include <QPaintEvent>
#include <QFontMetrics>

BatteryIndicator::BatteryIndicator(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(100, 40);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

BatteryIndicator::~BatteryIndicator() = default;

void BatteryIndicator::setBatteryLevel(double level)
{
    battery_level_ = qBound(0.0, level, 100.0);
    update();
}

QSize BatteryIndicator::sizeHint() const
{
    return QSize(150, 50);
}

QSize BatteryIndicator::minimumSizeHint() const
{
    return QSize(100, 40);
}

void BatteryIndicator::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawBattery(painter);
    drawBatteryLevel(painter);
    drawBatteryText(painter);
}

QColor BatteryIndicator::getBatteryColor() const
{
    if (battery_level_ > 50.0) {
        return QColor(0, 255, 0);  // 绿色
    } else if (battery_level_ > 20.0) {
        return QColor(255, 165, 0);  // 橙色
    } else {
        return QColor(255, 0, 0);  // 红色
    }
}

void BatteryIndicator::drawBattery(QPainter& painter)
{
    painter.save();

    // 绘制电池外框
    painter.setPen(QPen(border_color_, border_width_));
    painter.setBrush(background_color_);
    QRect battery_rect = rect().adjusted(
        border_width_ + terminal_width_,
        border_width_,
        -border_width_,
        -border_width_
    );
    painter.drawRect(battery_rect);

    // 绘制电池正极
    QRect terminal_rect(
        border_width_,
        height() / 2 - terminal_height_ / 2,
        terminal_width_,
        terminal_height_
    );
    painter.fillRect(terminal_rect, border_color_);

    painter.restore();
}

void BatteryIndicator::drawBatteryLevel(QPainter& painter)
{
    painter.save();

    // 计算电量条的宽度
    int level_width = static_cast<int>((width() - 2 * border_width_ - terminal_width_) * battery_level_ / 100.0);
    QRect level_rect(
        border_width_ + terminal_width_,
        border_width_,
        level_width,
        height() - 2 * border_width_
    );

    // 绘制电量条
    painter.setPen(Qt::NoPen);
    painter.setBrush(getBatteryColor());
    painter.drawRect(level_rect);

    painter.restore();
}

void BatteryIndicator::drawBatteryText(QPainter& painter)
{
    painter.save();

    QString text = QString("%1%").arg(static_cast<int>(battery_level_));
    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);

    QFontMetrics metrics(font);
    QRect text_rect = rect().adjusted(
        border_width_ + terminal_width_,
        border_width_,
        -border_width_,
        -border_width_
    );
    painter.drawText(text_rect, Qt::AlignCenter, text);

    painter.restore();
} 