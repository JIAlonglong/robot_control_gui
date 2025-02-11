#ifndef ACTION_COMMANDS_H
#define ACTION_COMMANDS_H

#include <QList>
#include <QPointF>
#include <QUndoCommand>
#include "action_block_widget.h"
#include "connection_line.h"

// 添加动作块命令
class AddBlockCommand : public QUndoCommand
{
public:
    AddBlockCommand(QGraphicsScene* scene, ActionBlockWidget* block, const QPointF& pos);
    ~AddBlockCommand() override;

    void undo() override;
    void redo() override;

private:
    QGraphicsScene*    scene_;
    ActionBlockWidget* block_;
    QPointF            pos_;
};

// 删除动作块命令
class RemoveBlockCommand : public QUndoCommand
{
public:
    RemoveBlockCommand(QGraphicsScene* scene, ActionBlockWidget* block);
    ~RemoveBlockCommand() override;

    void undo() override;
    void redo() override;

private:
    QGraphicsScene*        scene_;
    ActionBlockWidget*     block_;
    QPointF                pos_;
    QList<ConnectionLine*> connections_;
};

// 移动动作块命令
class MoveBlockCommand : public QUndoCommand
{
public:
    MoveBlockCommand(ActionBlockWidget* block, const QPointF& oldPos, const QPointF& newPos);

    void undo() override;
    void redo() override;

private:
    ActionBlockWidget* block_;
    QPointF            old_pos_;
    QPointF            new_pos_;
};

// 添加连接命令
class AddConnectionCommand : public QUndoCommand
{
public:
    AddConnectionCommand(QGraphicsScene* scene, ActionBlockWidget* from, ActionBlockWidget* to);
    ~AddConnectionCommand() override;

    void undo() override;
    void redo() override;

private:
    QGraphicsScene*    scene_;
    ActionBlockWidget* from_;
    ActionBlockWidget* to_;
    ConnectionLine*    line_;
};

// 删除连接命令
class RemoveConnectionCommand : public QUndoCommand
{
public:
    RemoveConnectionCommand(QGraphicsScene* scene, ConnectionLine* line);
    ~RemoveConnectionCommand() override;

    void undo() override;
    void redo() override;

private:
    QGraphicsScene*    scene_;
    ConnectionLine*    line_;
    ActionBlockWidget* from_;
    ActionBlockWidget* to_;
};

#endif  // ACTION_COMMANDS_H