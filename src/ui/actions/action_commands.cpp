/**
 * Copyright (c) 2025 JIAlonglong
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
 * @file action_commands.cpp
 * @brief 未知组件
 * @author JIAlonglong
 */

#include "action_commands.h"
#include <QGraphicsScene>

// AddBlockCommand 实现
AddBlockCommand::AddBlockCommand(QGraphicsScene* scene, ActionBlockWidget* block,
                                 const QPointF& pos)
    : scene_(scene), block_(block), pos_(pos)
{
    setText(QObject::tr("添加动作块"));
}

AddBlockCommand::~AddBlockCommand() = default;

void AddBlockCommand::undo()
{
    if (scene_ && block_) {
        scene_->removeItem(block_);
    }
}

void AddBlockCommand::redo()
{
    if (scene_ && block_) {
        block_->setPos(pos_);
        scene_->addItem(block_);
    }
}

// RemoveBlockCommand 实现
RemoveBlockCommand::RemoveBlockCommand(QGraphicsScene* scene, ActionBlockWidget* block)
    : scene_(scene), block_(block), pos_(block->pos())
{
    setText(QObject::tr("删除动作块"));

    // 保存相关的连接线
    for (QGraphicsItem* item : scene->items()) {
        if (ConnectionLine* line = qgraphicsitem_cast<ConnectionLine*>(item)) {
            if (line->fromBlock() == block || line->toBlock() == block) {
                connections_.append(line);
            }
        }
    }
}

RemoveBlockCommand::~RemoveBlockCommand() = default;

void RemoveBlockCommand::undo()
{
    if (scene_ && block_) {
        block_->setPos(pos_);
        scene_->addItem(block_);

        // 恢复连接线
        for (ConnectionLine* line : connections_) {
            scene_->addItem(line);
        }
    }
}

void RemoveBlockCommand::redo()
{
    if (scene_ && block_) {
        // 移除连接线
        for (ConnectionLine* line : connections_) {
            scene_->removeItem(line);
        }

        scene_->removeItem(block_);
    }
}

// MoveBlockCommand 实现
MoveBlockCommand::MoveBlockCommand(ActionBlockWidget* block, const QPointF& oldPos,
                                   const QPointF& newPos)
    : block_(block), old_pos_(oldPos), new_pos_(newPos)
{
    setText(QObject::tr("移动动作块"));
}

void MoveBlockCommand::undo()
{
    if (block_) {
        block_->setPos(old_pos_);
    }
}

void MoveBlockCommand::redo()
{
    if (block_) {
        block_->setPos(new_pos_);
    }
}

// AddConnectionCommand 实现
AddConnectionCommand::AddConnectionCommand(QGraphicsScene* scene, ActionBlockWidget* from,
                                           ActionBlockWidget* to)
    : scene_(scene), from_(from), to_(to), line_(new ConnectionLine(from, to))
{
    setText(QObject::tr("添加连接"));
}

AddConnectionCommand::~AddConnectionCommand() = default;

void AddConnectionCommand::undo()
{
    if (scene_ && line_) {
        scene_->removeItem(line_);
    }
}

void AddConnectionCommand::redo()
{
    if (scene_ && line_) {
        scene_->addItem(line_);
    }
}

// RemoveConnectionCommand 实现
RemoveConnectionCommand::RemoveConnectionCommand(QGraphicsScene* scene, ConnectionLine* line)
    : scene_(scene), line_(line), from_(line->fromBlock()), to_(line->toBlock())
{
    setText(QObject::tr("删除连接"));
}

RemoveConnectionCommand::~RemoveConnectionCommand() = default;

void RemoveConnectionCommand::undo()
{
    if (scene_ && line_) {
        scene_->addItem(line_);
    }
}

void RemoveConnectionCommand::redo()
{
    if (scene_ && line_) {
        scene_->removeItem(line_);
    }
}