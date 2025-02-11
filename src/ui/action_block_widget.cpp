#include "ui/action_block_widget.h"
#include "ui/action_block.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QDrag>
#include <QMimeData>
#include <QCursor>
#include <QApplication>
#include <QDebug>

ActionBlockWidget::ActionBlockWidget(ActionBlock* action, QGraphicsItem* parent)
    : QGraphicsObject(parent)
    , action_(action)
    , rect_(-50, -30, 100, 60)
{
    setAcceptHoverEvents(true);
    setAcceptDrops(true);
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);

    if (action_) {
        // 将action的父对象设置为this所属的QWidget
        if (scene()) {
            QList<QGraphicsView*> views = scene()->views();
            if (!views.isEmpty()) {
                if (QWidget* widget = views.first()) {
                    action_->setParent(widget);
                }
            }
        }
        connect(action_, &ActionBlock::nameChanged, this, [this]() { update(); });
        connect(action_, &ActionBlock::runningChanged, this, &ActionBlockWidget::updateState);
        connect(action_, &ActionBlock::started, this, &ActionBlockWidget::updateState);
        connect(action_, &ActionBlock::stopped, this, &ActionBlockWidget::updateState);
        connect(action_, &ActionBlock::completed, this, &ActionBlockWidget::updateState);
    }
}

ActionBlockWidget::~ActionBlockWidget()
{
    // action_的删除会由Qt的父子关系自动处理
}

QRectF ActionBlockWidget::boundingRect() const
{
    return rect_;
}

void ActionBlockWidget::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    // 绘制背景
    QColor bgColor = QColor(Qt::white);
    if (isSelected()) {
        bgColor = QColor(Qt::lightGray);
    } else if (is_hover_) {
        bgColor = QColor(Qt::lightGray).lighter();
    }

    if (is_running_) {
        bgColor = QColor(Qt::green).lighter();
    }

    painter->setBrush(bgColor);
    painter->setPen(QPen(Qt::black, 2));
    painter->drawRoundedRect(rect_, 10, 10);

    // 绘制图标
    if (action_) {
        QIcon icon = action_->icon();
        if (!icon.isNull()) {
            QPixmap pixmap = icon.pixmap(32, 32);
            painter->drawPixmap(-16, -16, pixmap);
        }

        // 绘制文本
        painter->setPen(Qt::black);
        painter->drawText(rect_, Qt::AlignCenter, action_->name());
    }

    // 绘制连接点
    painter->setBrush(Qt::white);
    painter->setPen(QPen(Qt::black, 1));
    painter->drawEllipse(inputPoint(), 5, 5);
    painter->drawEllipse(outputPoint(), 5, 5);
}

QPointF ActionBlockWidget::inputPoint() const
{
    return QPointF(rect_.left(), 0);
}

QPointF ActionBlockWidget::outputPoint() const
{
    return QPointF(rect_.right(), 0);
}

void ActionBlockWidget::updateState()
{
    if (action_) {
        is_running_ = action_->isRunning();
        update();
    }
}

void ActionBlockWidget::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        if (isOverOutputPoint(event->pos())) {
            is_connecting_ = true;
            emit connectionStarted(this, event->scenePos());
            event->accept();
            return;
        }
        drag_start_ = event->pos();
        is_dragging_ = true;
    }
    QGraphicsObject::mousePressEvent(event);
}

void ActionBlockWidget::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    if (is_connecting_) {
        // TODO: 实现连接线的绘制
        event->accept();
        return;
    }
    if (is_dragging_) {
        QPointF delta = event->pos() - drag_start_;
        if (delta.manhattanLength() >= QApplication::startDragDistance()) {
            // TODO: 实现拖拽
        }
    }
    QGraphicsObject::mouseMoveEvent(event);
}

void ActionBlockWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        if (is_connecting_) {
            is_connecting_ = false;
            emit connectionFinished(this, event->scenePos());
            event->accept();
            return;
        }
        is_dragging_ = false;
        emit selected(this);
    }
    QGraphicsObject::mouseReleaseEvent(event);
}

void ActionBlockWidget::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    is_hover_ = true;
    update();
    QGraphicsObject::hoverEnterEvent(event);
}

void ActionBlockWidget::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    is_hover_ = false;
    update();
    QGraphicsObject::hoverLeaveEvent(event);
}

void ActionBlockWidget::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
{
    hover_pos_ = event->pos();
    if (isOverInputPoint(hover_pos_) || isOverOutputPoint(hover_pos_)) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
    QGraphicsObject::hoverMoveEvent(event);
}

void ActionBlockWidget::dragEnterEvent(QGraphicsSceneDragDropEvent* event)
{
    if (event->mimeData()->hasFormat("application/x-action-block")) {
        event->acceptProposedAction();
    }
}

void ActionBlockWidget::dragLeaveEvent(QGraphicsSceneDragDropEvent* event)
{
    QGraphicsObject::dragLeaveEvent(event);
}

void ActionBlockWidget::dropEvent(QGraphicsSceneDragDropEvent* event)
{
    if (event->mimeData()->hasFormat("application/x-action-block")) {
        // TODO: 实现动作块的放置
        event->acceptProposedAction();
    }
}

QVariant ActionBlockWidget::itemChange(GraphicsItemChange change, const QVariant& value)
{
    if (change == ItemPositionChange && scene()) {
        emit positionChanged(value.toPointF());
    }
    return QGraphicsObject::itemChange(change, value);
}

bool ActionBlockWidget::isOverInputPoint(const QPointF& pos) const
{
    return QPointF(inputPoint() - pos).manhattanLength() < 10;
}

bool ActionBlockWidget::isOverOutputPoint(const QPointF& pos) const
{
    return QPointF(outputPoint() - pos).manhattanLength() < 10;
}

#include "action_block_widget.moc" 