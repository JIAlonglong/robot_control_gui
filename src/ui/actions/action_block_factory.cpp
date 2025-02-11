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
 * @file action_block_factory.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "action_block_factory.h"
#include <QDebug>
#include <QIcon>
#include <stdexcept>
#include "robot_controller.h"
#include "condition_action_block.h"
#include "loop_action_block.h"
#include "move_action_block.h"
#include "movement_action_block.h"
#include "navigation_action_block.h"
#include "path_action_block.h"
#include "wait_action_block.h"

struct ActionBlockFactory::Private {
    std::shared_ptr<RobotController>                     robot_controller;
    QMap<QString, std::function<ActionBlock*(QObject*)>> creators;
};

ActionBlockFactory::ActionBlockFactory(const std::shared_ptr<RobotController>& controller,
                                       QObject*                                parent)
    : QObject(parent), d_(std::make_unique<Private>())
{
    if (!controller) {
        throw std::invalid_argument("机器人控制器不能为空");
    }
    d_->robot_controller = controller;

    // 注册动作块创建器
    registerActionType("move", [this](QObject* parent) -> ActionBlock* {
        auto* block = new MoveActionBlock(d_->robot_controller, parent);
        return block;
    });

    registerActionType("path", [this](QObject* parent) -> ActionBlock* {
        auto* block = new PathActionBlock(d_->robot_controller, parent);
        return block;
    });

    registerActionType("wait", [this](QObject* parent) -> ActionBlock* {
        auto* block = new WaitActionBlock(parent);
        return block;
    });

    registerActionType("loop", [this](QObject* parent) -> ActionBlock* {
        auto* block = new LoopActionBlock(parent);
        return block;
    });

    registerActionType("condition", [this](QObject* parent) -> ActionBlock* {
        auto* block = new ConditionActionBlock(d_->robot_controller, parent);
        return block;
    });

    registerActionType("navigation", [this](QObject* parent) -> ActionBlock* {
        return new NavigationActionBlock(d_->robot_controller, parent);
    });
}

ActionBlockFactory::~ActionBlockFactory() = default;

QStringList ActionBlockFactory::availableTypes() const
{
    return {"move", "path", "wait", "loop", "condition", "navigation"};
}

ActionBlock* ActionBlockFactory::create(const QString& type, QObject* parent)
{
    if (type.isEmpty()) {
        throw std::invalid_argument("动作类型不能为空");
    }

    auto it = d_->creators.find(type);
    if (it != d_->creators.end()) {
        try {
            return it.value()(parent);
        } catch (const std::exception& e) {
            qWarning() << "创建动作块失败:" << e.what();
            Q_EMIT creationFailed(type, e.what());
            return nullptr;
        }
    }

    QString error = QString("不支持的动作类型: %1").arg(type);
    qWarning() << error;
    Q_EMIT creationFailed(type, error);
    return nullptr;
}

ActionBlock* ActionBlockFactory::createAction(const QString& type)
{
    if (!d_->robot_controller) {
        return nullptr;
    }

    if (type == "movement") {
        return new MovementActionBlock(d_->robot_controller, this);
    } else if (type == "path") {
        return new PathActionBlock(d_->robot_controller, this);
    } else if (type == "navigation") {
        return new NavigationActionBlock(d_->robot_controller, this);
    } else if (type == "condition") {
        return new ConditionActionBlock(d_->robot_controller, this);
    }

    return nullptr;
}

void ActionBlockFactory::setRobotController(std::shared_ptr<RobotController> controller)
{
    if (!controller) {
        throw std::invalid_argument("机器人控制器不能为空");
    }
    d_->robot_controller = controller;
}

void ActionBlockFactory::registerActionType(const QString&                        type,
                                            std::function<ActionBlock*(QObject*)> creator)
{
    if (type.isEmpty()) {
        throw std::invalid_argument("动作类型不能为空");
    }
    if (!creator) {
        throw std::invalid_argument("创建函数不能为空");
    }

    d_->creators[type] = std::move(creator);
    Q_EMIT actionTypeRegistered(type);
}

QString ActionBlockFactory::actionName(const QString& type) const
{
    if (type == "move") {
        return tr("移动动作");
    } else if (type == "path") {
        return tr("路径动作");
    } else if (type == "wait") {
        return tr("等待动作");
    } else if (type == "loop") {
        return tr("循环动作");
    } else if (type == "condition") {
        return tr("条件动作");
    } else if (type == "navigation") {
        return tr("导航动作");
    }
    return QString();
}

QString ActionBlockFactory::actionDescription(const QString& type) const
{
    if (type == "move") {
        return tr("控制机器人按指定速度运动指定时间");
    } else if (type == "path") {
        return tr("控制机器人沿预定义路径运动");
    } else if (type == "wait") {
        return tr("控制机器人等待一段时间");
    } else if (type == "loop") {
        return tr("控制机器人重复执行动作");
    } else if (type == "condition") {
        return tr("根据条件执行不同的动作");
    } else if (type == "navigation") {
        return tr("控制机器人导航到指定位置");
    }
    return QString();
}

QIcon ActionBlockFactory::actionIcon(const QString& type) const
{
    if (type == "move") {
        return QIcon(":/icons/movement.png");
    } else if (type == "path") {
        return QIcon(":/icons/path.png");
    } else if (type == "wait") {
        return QIcon(":/icons/wait.png");
    } else if (type == "loop") {
        return QIcon(":/icons/loop.png");
    } else if (type == "condition") {
        return QIcon(":/icons/condition.png");
    } else if (type == "navigation") {
        return QIcon(":/icons/navigation.png");
    }
    return QIcon();
}

#include "action_block_factory.moc"