#ifndef CONNECTION_LINE_H
#define CONNECTION_LINE_H

#include <QGraphicsPathItem>
#include "action_block_widget.h"

class ConnectionLine : public QGraphicsPathItem
{
public:
    ConnectionLine(ActionBlockWidget* from, ActionBlockWidget* to, QGraphicsItem* parent = nullptr);

    ActionBlockWidget* fromBlock() const
    {
        return from_;
    }
    ActionBlockWidget* toBlock() const
    {
        return to_;
    }

    void updatePosition();

protected:
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget = nullptr) override;

private:
    ActionBlockWidget* from_;
    ActionBlockWidget* to_;
};

#endif  // CONNECTION_LINE_H