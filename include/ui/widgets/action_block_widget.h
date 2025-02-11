#ifndef ACTION_BLOCK_WIDGET_H
#define ACTION_BLOCK_WIDGET_H

#include <QGraphicsObject>
#include <QMimeData>
#include <QPainter>
#include <QVariant>
#include "action_block.h"

class ActionBlockWidget : public QGraphicsObject
{
    Q_OBJECT

public:
    explicit ActionBlockWidget(ActionBlock* action, QGraphicsItem* parent = nullptr);
    ~ActionBlockWidget() override;

    QRectF boundingRect() const override;
    void   paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                 QWidget* widget = nullptr) override;

    ActionBlock* action() const
    {
        return action_;
    }
    QPointF inputPoint() const;
    QPointF outputPoint() const;
    void    updateState();

Q_SIGNALS:
    void selected(ActionBlockWidget* block);
    void connectionStarted(ActionBlockWidget* block, const QPointF& point);
    void connectionFinished(ActionBlockWidget* block, const QPointF& point);
    void positionChanged(const QPointF& pos);
    void deselected(ActionBlockWidget* block);
    void removed(ActionBlockWidget* block);
    void propertyChanged(const QString& name, const QVariant& value);

public Q_SLOTS:
    void setProperty(const QString& name, const QVariant& value);
    void setSelected(bool selected);
    void remove();

protected:
    void     mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void     mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void     mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void     hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
    void     hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
    void     hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;
    void     dragEnterEvent(QGraphicsSceneDragDropEvent* event) override;
    void     dragLeaveEvent(QGraphicsSceneDragDropEvent* event) override;
    void     dropEvent(QGraphicsSceneDragDropEvent* event) override;
    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

private:
    bool isOverInputPoint(const QPointF& pos) const;
    bool isOverOutputPoint(const QPointF& pos) const;

    ActionBlock* action_;
    QRectF       rect_;
    bool         is_running_{false};
    bool         is_dragging_{false};
    bool         is_connecting_{false};
    bool         is_hover_{false};
    QPointF      drag_start_;
    QPointF      hover_pos_;
};

#endif  // ACTION_BLOCK_WIDGET_H