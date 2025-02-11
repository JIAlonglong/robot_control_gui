#include "ui/action_block.h"
#include <QDebug>
#include <QJsonObject>
#include <QJsonArray>
#include <stdexcept>

struct ActionBlock::Private {
    QString category;
    bool is_running{false};
    bool is_paused{false};
    QPoint position;
    QPoint input_point;
    QPoint output_point;
    QMap<QString, QVariant> parameters;
};

ActionBlock::ActionBlock(QObject* parent)
    : QObject(parent)
    , d_(std::make_unique<Private>())
{
}

ActionBlock::~ActionBlock() = default;

bool ActionBlock::isRunning() const
{
    return d_->is_running;
}

void ActionBlock::reset()
{
    if (d_->is_running) {
        stop();
    }
    d_->is_running = false;
    d_->is_paused = false;
    emit runningChanged(false);
}

QString ActionBlock::category() const
{
    return d_->category;
}

void ActionBlock::setCategory(const QString& category)
{
    if (d_->category != category) {
        d_->category = category;
        emit categoryChanged(category);
    }
}

bool ActionBlock::validateParameters() const
{
    return true;
}

QJsonObject ActionBlock::toJson() const
{
    QJsonObject json;
    json["type"] = type();
    json["name"] = name();
    json["description"] = description();
    json["category"] = category();
    json["position"] = QJsonObject{
        {"x", d_->position.x()},
        {"y", d_->position.y()}
    };
    json["input_point"] = QJsonObject{
        {"x", d_->input_point.x()},
        {"y", d_->input_point.y()}
    };
    json["output_point"] = QJsonObject{
        {"x", d_->output_point.x()},
        {"y", d_->output_point.y()}
    };

    QJsonObject params;
    for (auto it = d_->parameters.constBegin(); it != d_->parameters.constEnd(); ++it) {
        params[it.key()] = QJsonValue::fromVariant(it.value());
    }
    json["parameters"] = params;

    return json;
}

void ActionBlock::fromJson(const QJsonObject& json)
{
    try {
        if (json["type"].toString() != type()) {
            throw std::runtime_error("类型不匹配");
        }

        setCategory(json["category"].toString());

        auto pos = json["position"].toObject();
        setPosition(QPoint(pos["x"].toInt(), pos["y"].toInt()));

        auto input = json["input_point"].toObject();
        setInputPoint(QPoint(input["x"].toInt(), input["y"].toInt()));

        auto output = json["output_point"].toObject();
        setOutputPoint(QPoint(output["x"].toInt(), output["y"].toInt()));

        auto params = json["parameters"].toObject();
        for (auto it = params.begin(); it != params.end(); ++it) {
            setParameter(it.key(), it.value().toVariant());
        }

    } catch (const std::exception& e) {
        emit error(tr("从JSON加载失败: %1").arg(e.what()));
    }
}

QPoint ActionBlock::position() const
{
    return d_->position;
}

void ActionBlock::setPosition(const QPoint& pos)
{
    if (d_->position != pos) {
        d_->position = pos;
        emit positionChanged(pos);
    }
}

QVariant ActionBlock::parameter(const QString& key) const
{
    return d_->parameters.value(key);
}

void ActionBlock::setParameter(const QString& key, const QVariant& value)
{
    if (d_->parameters[key] != value) {
        d_->parameters[key] = value;
        emit parameterChanged(key, value);
    }
}

QMap<QString, QVariant> ActionBlock::parameters() const
{
    return d_->parameters;
}

void ActionBlock::setParameters(const QMap<QString, QVariant>& params)
{
    if (d_->parameters != params) {
        d_->parameters = params;
        for (auto it = params.constBegin(); it != params.constEnd(); ++it) {
            emit parameterChanged(it.key(), it.value());
        }
    }
}

QPoint ActionBlock::inputPoint() const
{
    return d_->input_point;
}

QPoint ActionBlock::outputPoint() const
{
    return d_->output_point;
}

void ActionBlock::setInputPoint(const QPoint& point)
{
    if (d_->input_point != point) {
        d_->input_point = point;
        emit inputPointChanged(point);
    }
}

void ActionBlock::setOutputPoint(const QPoint& point)
{
    if (d_->output_point != point) {
        d_->output_point = point;
        emit outputPointChanged(point);
    }
}

QVariantMap ActionBlock::toVariantMap() const
{
    QVariantMap map;
    map["type"] = type();
    map["name"] = name();
    map["description"] = description();
    map["category"] = category();
    map["position"] = QVariantMap{
        {"x", d_->position.x()},
        {"y", d_->position.y()}
    };
    map["input_point"] = QVariantMap{
        {"x", d_->input_point.x()},
        {"y", d_->input_point.y()}
    };
    map["output_point"] = QVariantMap{
        {"x", d_->output_point.x()},
        {"y", d_->output_point.y()}
    };
    map["parameters"] = QVariant::fromValue(d_->parameters);
    return map;
}

void ActionBlock::fromVariantMap(const QVariantMap& map)
{
    try {
        if (map["type"].toString() != type()) {
            throw std::runtime_error("类型不匹配");
        }

        setCategory(map["category"].toString());

        auto pos = map["position"].toMap();
        setPosition(QPoint(pos["x"].toInt(), pos["y"].toInt()));

        auto input = map["input_point"].toMap();
        setInputPoint(QPoint(input["x"].toInt(), input["y"].toInt()));

        auto output = map["output_point"].toMap();
        setOutputPoint(QPoint(output["x"].toInt(), output["y"].toInt()));

        setParameters(map["parameters"].toMap());

    } catch (const std::exception& e) {
        emit error(tr("从VariantMap加载失败: %1").arg(e.what()));
    }
}

void ActionBlock::pause()
{
    if (d_->is_running && !d_->is_paused) {
        d_->is_paused = true;
        onExecutionPaused();
        emit paused();
    }
}

void ActionBlock::resume()
{
    if (d_->is_running && d_->is_paused) {
        d_->is_paused = false;
        onExecutionResumed();
        emit resumed();
    }
}

void ActionBlock::validateConnection(ActionBlock* target)
{
    if (!target) {
        throw std::invalid_argument("目标动作块不能为空");
    }
}

void ActionBlock::onExecutionStarted()
{
    d_->is_running = true;
    d_->is_paused = false;
    emit runningChanged(true);
    emit started();
}

void ActionBlock::onExecutionFinished(bool success)
{
    d_->is_running = false;
    d_->is_paused = false;
    emit runningChanged(false);
    emit completed(success);
}

void ActionBlock::onExecutionError(const QString& error_msg)
{
    d_->is_running = false;
    emit error(error_msg);
    emit failed(error_msg);
}

void ActionBlock::onExecutionStopped()
{
    d_->is_running = false;
    d_->is_paused = false;
    emit runningChanged(false);
    emit stopped();
}

void ActionBlock::onExecutionPaused()
{
    emit statusChanged(tr("已暂停"));
}

void ActionBlock::onExecutionResumed()
{
    emit statusChanged(tr("已恢复"));
}

#include "ui/action_block.moc" 