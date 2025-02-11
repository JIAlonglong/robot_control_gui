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
 * @file action_sequence_manager.cpp
 * @brief 动作序列管理器,负责管理和执行动作序列
 * @author JIAlonglong
 */

#include "action_sequence_manager.h"
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

struct ActionSequenceManager::Private {
    std::vector<ActionBlock*> actions;
    bool                      is_running{false};
    bool                      is_paused{false};
    int                       current_index{-1};
    ActionBlock*              current_action{nullptr};
    ActionBlockFactory*       factory{nullptr};
};

ActionSequenceManager::ActionSequenceManager(QObject* parent)
    : QObject(parent), d_(std::make_unique<Private>())
{
}

ActionSequenceManager::~ActionSequenceManager() = default;

void ActionSequenceManager::setActionBlockFactory(ActionBlockFactory* factory)
{
    d_->factory = factory;
}

void ActionSequenceManager::addAction(const QString& type)
{
    if (!d_->factory) {
        return;
    }

    ActionBlock* action = d_->factory->create(type, this);
    if (action) {
        d_->actions.push_back(action);
        connect(action, &ActionBlock::started, this,
                [this, action]() { emit actionStarted(action); });
        connect(action, &ActionBlock::completed, this, [this, action]() {
            emit actionCompleted(action);
            onActionCompleted(true);
        });
        connect(action, &ActionBlock::failed, this, [this, action](const QString& error) {
            emit actionFailed(action, error);
            onActionCompleted(false);
        });
        connect(action, &ActionBlock::statusUpdated, this,
                [this](const QString& status) { emit statusUpdated(status); });

        emit actionAdded(action);
    }
}

void ActionSequenceManager::removeAction(ActionBlock* action)
{
    if (!action) return;

    auto it = std::find(d_->actions.begin(), d_->actions.end(), action);
    if (it == d_->actions.end()) return;

    int index = std::distance(d_->actions.begin(), it);
    d_->actions.erase(it);
    emit actionRemoved(action);

    if (d_->current_index >= index) --d_->current_index;
}

void ActionSequenceManager::moveActionUp(ActionBlock* action)
{
    if (!action) return;

    auto it = std::find(d_->actions.begin(), d_->actions.end(), action);
    if (it == d_->actions.end() || it == d_->actions.begin()) return;

    int old_index = std::distance(d_->actions.begin(), it);
    int new_index = old_index - 1;
    std::iter_swap(it, it - 1);
    emit actionMoved(action, old_index, new_index);

    if (d_->current_index == old_index)
        --d_->current_index;
    else if (d_->current_index == new_index)
        ++d_->current_index;
}

void ActionSequenceManager::moveActionDown(ActionBlock* action)
{
    if (!action) return;

    auto it = std::find(d_->actions.begin(), d_->actions.end(), action);
    if (it == d_->actions.end() || it + 1 == d_->actions.end()) return;

    int old_index = std::distance(d_->actions.begin(), it);
    int new_index = old_index + 1;
    std::iter_swap(it, it + 1);
    emit actionMoved(action, old_index, new_index);

    if (d_->current_index == old_index)
        ++d_->current_index;
    else if (d_->current_index == new_index)
        --d_->current_index;
}

void ActionSequenceManager::clearActions()
{
    if (d_->is_running) stop();

    d_->actions.clear();
    d_->current_index  = -1;
    d_->current_action = nullptr;
}

const std::vector<ActionBlock*>& ActionSequenceManager::actions() const
{
    return d_->actions;
}

void ActionSequenceManager::start()
{
    if (d_->is_running) return;

    if (d_->actions.empty()) {
        emit error(tr("没有可执行的动作"));
        return;
    }

    d_->is_running    = true;
    d_->is_paused     = false;
    d_->current_index = -1;
    emit sequenceStarted();
    emit statusUpdated(tr("开始执行动作序列"));

    executeNextAction();
}

void ActionSequenceManager::stop()
{
    if (!d_->is_running) return;

    d_->is_running = false;
    d_->is_paused  = false;
    if (d_->current_action) d_->current_action->stop();

    d_->current_index  = -1;
    d_->current_action = nullptr;
    emit sequenceStopped();
    emit statusUpdated(tr("动作序列已停止"));
}

void ActionSequenceManager::pause()
{
    if (!d_->is_running || d_->is_paused) return;

    d_->is_paused = true;
    if (d_->current_action) d_->current_action->stop();

    emit sequencePaused();
    emit statusUpdated(tr("动作序列已暂停"));
}

void ActionSequenceManager::resume()
{
    if (!d_->is_running || !d_->is_paused) return;

    d_->is_paused = false;
    emit sequenceResumed();
    emit statusUpdated(tr("动作序列已恢复"));

    executeNextAction();
}

void ActionSequenceManager::executeNextAction()
{
    if (!d_->is_running || d_->is_paused) return;

    ++d_->current_index;
    if (d_->current_index >= static_cast<int>(d_->actions.size())) {
        d_->is_running     = false;
        d_->current_index  = -1;
        d_->current_action = nullptr;
        emit sequenceCompleted(true);
        emit statusUpdated(tr("动作序列执行完成"));
        return;
    }

    d_->current_action = d_->actions[d_->current_index];
    emit statusUpdated(tr("执行动作: %1").arg(d_->current_action->name()));
    d_->current_action->execute();
}

void ActionSequenceManager::onActionCompleted(bool success)
{
    if (!success) {
        stop();
        emit sequenceCompleted(false);
        return;
    }

    executeNextAction();
}

QJsonArray ActionSequenceManager::toJson() const
{
    QJsonArray array;
    for (const ActionBlock* action : d_->actions) {
        QJsonObject obj = action->toJson();
        obj["type"]     = action->type();
        array.append(obj);
    }
    return array;
}

void ActionSequenceManager::fromJson(const QJsonArray& json)
{
    if (!d_->factory) {
        qWarning() << "未设置动作块工厂，无法加载动作序列";
        return;
    }

    clearActions();
    for (const QJsonValue& value : json) {
        if (!value.isObject()) continue;

        QJsonObject obj  = value.toObject();
        QString     type = obj["type"].toString();

        auto* action = d_->factory->createAction(type);
        if (!action) {
            qWarning() << "无法创建动作类型:" << type;
            continue;
        }

        action->fromJson(obj);
        addAction(type);
    }
}

void ActionSequenceManager::saveToFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        emit error(tr("无法打开文件保存: %1").arg(filename));
        return;
    }

    QJsonObject root;
    root["version"] = "1.0";
    root["actions"] = toJson();

    QJsonDocument doc(root);
    if (file.write(doc.toJson()) == -1) {
        emit error(tr("保存文件失败: %1").arg(file.errorString()));
        return;
    }

    emit statusUpdated(tr("动作序列已保存到: %1").arg(filename));
}

void ActionSequenceManager::loadFromFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        emit error(tr("无法打开文件加载: %1").arg(filename));
        return;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        emit error(tr("无效的文件格式: %1").arg(filename));
        return;
    }

    QJsonObject root    = doc.object();
    QString     version = root["version"].toString();
    if (version != "1.0") {
        emit error(tr("不支持的文件版本: %1").arg(version));
        return;
    }

    fromJson(root["actions"].toArray());
    emit statusUpdated(tr("动作序列已加载自: %1").arg(filename));
}

void ActionSequenceManager::setActions(const QList<ActionBlock*>& actions)
{
    clearActions();
    for (auto* action : actions) {
        addAction(action->type());
    }
}

void ActionSequenceManager::onActionError(const QString& error_msg)
{
    if (d_->is_running) {
        stop();
        emit error(error_msg);
        emit sequenceCompleted(false);
    }
}