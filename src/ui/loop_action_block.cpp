#include "ui/loop_action_block.h"
#include <QDebug>

struct LoopActionBlock::Private {
    int count{1};
    int current_count{0};
    ActionBlock* action{nullptr};
    bool is_running{false};
};

LoopActionBlock::LoopActionBlock(QObject* parent)
    : ActionBlock(parent)
    , d_(std::make_unique<Private>())
{
}

LoopActionBlock::~LoopActionBlock() = default;

QString LoopActionBlock::type() const
{
    return "loop";
}

QString LoopActionBlock::name() const
{
    return tr("循环动作");
}

QString LoopActionBlock::description() const
{
    return tr("重复执行指定的动作指定次数");
}

QIcon LoopActionBlock::icon() const
{
    return QIcon::fromTheme("view-refresh");
}

bool LoopActionBlock::isValid() const
{
    return d_->action != nullptr && d_->count > 0;
}

bool LoopActionBlock::isRunning() const
{
    return d_->is_running;
}

int LoopActionBlock::count() const
{
    return d_->count;
}

void LoopActionBlock::setCount(int count)
{
    if (d_->count == count) {
        return;
    }
    d_->count = count;
    Q_EMIT countChanged(count);
}

ActionBlock* LoopActionBlock::action() const
{
    return d_->action;
}

void LoopActionBlock::setAction(ActionBlock* action)
{
    if (d_->action == action) {
        return;
    }

    if (d_->action) {
        disconnect(d_->action, &ActionBlock::completed,
                  this, &LoopActionBlock::onActionCompleted);
    }

    d_->action = action;

    if (d_->action) {
        connect(d_->action, &ActionBlock::completed,
               this, &LoopActionBlock::onActionCompleted);
    }

    Q_EMIT actionChanged(action);
}

QVariantMap LoopActionBlock::properties() const
{
    QVariantMap props;
    props["count"] = d_->count;
    props["action"] = QVariant::fromValue(d_->action);
    return props;
}

void LoopActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "count") {
        setCount(value.toInt());
    } else if (name == "action") {
        setAction(value.value<ActionBlock*>());
    }
    Q_EMIT propertyChanged(name, value);
}

void LoopActionBlock::execute()
{
    if (!d_->action) {
        Q_EMIT error(tr("未设置子动作"));
        Q_EMIT completed(false);
        return;
    }

    if (d_->is_running) {
        Q_EMIT error(tr("循环动作正在执行中"));
        Q_EMIT completed(false);
        return;
    }

    d_->current_count = 0;
    d_->is_running = true;
    Q_EMIT runningChanged(true);
    Q_EMIT started();
    d_->action->execute();
}

void LoopActionBlock::stop()
{
    if (d_->is_running) {
        if (d_->action) {
            d_->action->stop();
        }
        d_->is_running = false;
        Q_EMIT runningChanged(false);
        Q_EMIT stopped();
        Q_EMIT completed(false);
    }
}

void LoopActionBlock::reset()
{
    stop();
    d_->current_count = 0;
    if (d_->action) {
        d_->action->reset();
    }
}

void LoopActionBlock::onActionCompleted(bool success)
{
    if (!d_->is_running) {
        return;
    }

    if (!success) {
        d_->is_running = false;
        Q_EMIT runningChanged(false);
        Q_EMIT completed(false);
        return;
    }

    d_->current_count++;
    if (d_->current_count < d_->count) {
        d_->action->execute();
    } else {
        d_->is_running = false;
        Q_EMIT runningChanged(false);
        Q_EMIT completed(true);
    }
}

void LoopActionBlock::pause()
{
    if (d_->is_running && d_->action) {
        d_->action->pause();
        Q_EMIT paused();
    }
}

void LoopActionBlock::resume()
{
    if (d_->is_running && d_->action) {
        d_->action->resume();
        Q_EMIT resumed();
    }
}

QJsonObject LoopActionBlock::toJson() const
{
    QJsonObject json = ActionBlock::toJson();
    json["count"] = d_->count;
    if (d_->action) {
        json["action"] = d_->action->toJson();
    }
    return json;
}

void LoopActionBlock::fromJson(const QJsonObject& json)
{
    ActionBlock::fromJson(json);
    setCount(json["count"].toInt());
    if (json.contains("action")) {
        // TODO: 从JSON创建动作
    }
}
