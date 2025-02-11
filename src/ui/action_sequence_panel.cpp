#include "ui/action_sequence_panel.h"
#include "ui/action_block.h"
#include "ui/action_block_factory.h"
#include "ui/action_sequence_manager.h"
#include "ui/action_selection_dialog.h"
#include "ui/property_editor.h"
#include "ui/action_list_widget.h"
#include "ui/action_configurator.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolBar>
#include <QListWidget>
#include <QPushButton>
#include <QMessageBox>
#include <QDebug>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

struct ActionSequencePanel::Private {
    std::shared_ptr<RobotController> controller;
    std::shared_ptr<ActionBlockFactory> factory;
    ActionSequenceManager* manager{nullptr};

    QVBoxLayout* layout{nullptr};
    QToolBar* toolbar{nullptr};
    QListWidget* action_list{nullptr};
    PropertyEditor* property_editor{nullptr};
    QComboBox* type_combo{nullptr};

    QPushButton* add_button{nullptr};
    QPushButton* remove_button{nullptr};
    QPushButton* up_button{nullptr};
    QPushButton* down_button{nullptr};
    QPushButton* start_button{nullptr};
    QPushButton* stop_button{nullptr};
    QPushButton* pause_button{nullptr};

    ActionBlock* current_action{nullptr};

    QList<ActionBlock*> actions;
    int current_action_index{-1};
    bool is_running{false};
    bool is_paused{false};

    ActionConfigurator* configurator{nullptr};
};

ActionSequencePanel::ActionSequencePanel(QWidget* parent)
    : QWidget(parent)
    , d_(std::make_unique<Private>())
{
    setupUi();
}

ActionSequencePanel::~ActionSequencePanel() = default;

void ActionSequencePanel::setActionBlockFactory(const std::shared_ptr<ActionBlockFactory>& factory)
{
    d_->factory = factory;
    if (d_->type_combo && factory) {
        d_->type_combo->clear();
        for (const auto& type : d_->factory->availableTypes()) {
            d_->type_combo->addItem(d_->factory->actionName(type), type);
        }
    }
    updateButtons();
}

void ActionSequencePanel::setActionSequenceManager(ActionSequenceManager* manager)
{
    if (d_->manager) {
        disconnect(d_->manager, nullptr, this, nullptr);
    }

    d_->manager = manager;
    if (manager) {
        connect(manager, &ActionSequenceManager::started, this, &ActionSequencePanel::onSequenceStarted);
        connect(manager, &ActionSequenceManager::stopped, this, &ActionSequencePanel::onSequenceStopped);
        connect(manager, &ActionSequenceManager::completed, this, &ActionSequencePanel::onSequenceCompleted);
        connect(manager, &ActionSequenceManager::error, this, [this](const QString& error) {
            QMessageBox::warning(this, tr("错误"), error);
        });
    }
    updateButtons();
}

void ActionSequencePanel::setupUi()
{
    d_->layout = new QVBoxLayout(this);
    d_->layout->setContentsMargins(0, 0, 0, 0);
    d_->layout->setSpacing(0);

    // 创建工具栏
    auto* toolbar = new QHBoxLayout;
    toolbar->setContentsMargins(0, 0, 0, 0);
    toolbar->setSpacing(6);

    d_->add_button = new QPushButton(tr("添加"), this);
    d_->remove_button = new QPushButton(tr("删除"), this);
    d_->up_button = new QPushButton(tr("上移"), this);
    d_->down_button = new QPushButton(tr("下移"), this);
    d_->start_button = new QPushButton(tr("执行"), this);
    d_->stop_button = new QPushButton(tr("停止"), this);
    d_->pause_button = new QPushButton(tr("暂停"), this);

    toolbar->addWidget(d_->add_button);
    toolbar->addWidget(d_->remove_button);
    toolbar->addWidget(d_->up_button);
    toolbar->addWidget(d_->down_button);
    toolbar->addWidget(d_->start_button);
    toolbar->addWidget(d_->stop_button);
    toolbar->addWidget(d_->pause_button);
    toolbar->addStretch();

    d_->layout->addLayout(toolbar);

    // 创建动作列表
    d_->action_list = new ActionListWidget(this);
    d_->layout->addWidget(d_->action_list);

    // 创建动作配置器
    d_->configurator = new ActionConfigurator(d_->factory, this);
    d_->layout->addWidget(d_->configurator);

    // 连接信号和槽
    connect(d_->add_button, &QPushButton::clicked, this, &ActionSequencePanel::onAddAction);
    connect(d_->remove_button, &QPushButton::clicked, this, &ActionSequencePanel::onRemoveAction);
    connect(d_->up_button, &QPushButton::clicked, this, &ActionSequencePanel::onMoveUpAction);
    connect(d_->down_button, &QPushButton::clicked, this, &ActionSequencePanel::onMoveDownAction);
    connect(d_->start_button, &QPushButton::clicked, this, &ActionSequencePanel::execute);
    connect(d_->stop_button, &QPushButton::clicked, this, &ActionSequencePanel::stop);
    connect(d_->pause_button, &QPushButton::clicked, this, &ActionSequencePanel::pause);

    connect(d_->action_list, &ActionListWidget::currentRowChanged,
            this, &ActionSequencePanel::onActionSelected);

    updateButtons();
}

void ActionSequencePanel::updateButtons()
{
    bool has_factory = d_->factory != nullptr;
    bool has_actions = !d_->actions.isEmpty();
    bool has_selection = d_->current_action != nullptr;
    bool is_first = d_->action_list && d_->action_list->currentRow() == 0;
    bool is_last = d_->action_list && d_->action_list->currentRow() == d_->action_list->count() - 1;

    d_->add_button->setEnabled(has_factory && !d_->is_running);
    d_->remove_button->setEnabled(has_selection && !d_->is_running);
    d_->up_button->setEnabled(has_selection && !is_first && !d_->is_running);
    d_->down_button->setEnabled(has_selection && !is_last && !d_->is_running);
    d_->start_button->setEnabled(has_actions && !d_->is_running);
    d_->stop_button->setEnabled(d_->is_running);
    d_->pause_button->setEnabled(d_->is_running && !d_->is_paused);
}

void ActionSequencePanel::execute()
{
    if (d_->is_running || d_->actions.isEmpty()) {
        return;
    }

    d_->is_running = true;
    d_->is_paused = false;
    d_->current_action_index = 0;
    d_->current_action = d_->actions[d_->current_action_index];

    Q_EMIT sequenceStarted();
    executeNext();
    updateButtons();
}

void ActionSequencePanel::stop()
{
    if (!d_->is_running) {
        return;
    }

    if (d_->current_action_index >= 0 && d_->current_action_index < d_->actions.size()) {
        d_->actions[d_->current_action_index]->stop();
    }

    d_->is_running = false;
    d_->is_paused = false;
    d_->current_action_index = -1;
    d_->current_action = nullptr;

    Q_EMIT sequenceStopped();
    updateButtons();
}

void ActionSequencePanel::pause()
{
    if (!d_->is_running || d_->is_paused) {
        return;
    }

    if (d_->current_action_index >= 0 && d_->current_action_index < d_->actions.size()) {
        d_->actions[d_->current_action_index]->pause();
    }

    d_->is_paused = true;
    Q_EMIT sequencePaused();
    updateButtons();
}

void ActionSequencePanel::resume()
{
    if (!d_->is_running || !d_->is_paused) {
        return;
    }

    if (d_->current_action_index >= 0 && d_->current_action_index < d_->actions.size()) {
        d_->actions[d_->current_action_index]->resume();
    }

    d_->is_paused = false;
    Q_EMIT sequenceResumed();
    updateButtons();
}

void ActionSequencePanel::onAddAction()
{
    if (!d_->factory) {
        return;
    }

    QString type = d_->type_combo->currentData().toString();
    if (type.isEmpty()) {
        return;
    }

    auto* action = d_->factory->create(type, this);
    if (!action) {
        return;
    }

    connectAction(action);
    d_->actions.append(action);

    auto* item = new QListWidgetItem(action->name());
    item->setData(Qt::UserRole, QVariant::fromValue(action));
    d_->action_list->addItem(item);
    d_->action_list->setCurrentItem(item);

    Q_EMIT actionAdded(d_->actions.size() - 1);
    updateButtons();
}

void ActionSequencePanel::onRemoveAction()
{
    int row = d_->action_list->currentRow();
    if (row < 0 || row >= d_->actions.size()) {
        return;
    }

    auto* action = d_->actions.takeAt(row);
    disconnectAction(action);
    action->deleteLater();

    delete d_->action_list->takeItem(row);

    Q_EMIT actionRemoved(row);
    updateButtons();
}

void ActionSequencePanel::onMoveUpAction()
{
    int row = d_->action_list->currentRow();
    if (row <= 0 || row >= d_->actions.size()) {
        return;
    }

    d_->actions.swapItemsAt(row, row - 1);
    auto* item = d_->action_list->takeItem(row);
    d_->action_list->insertItem(row - 1, item);
    d_->action_list->setCurrentItem(item);

    Q_EMIT actionMoved(row, row - 1);
    updateButtons();
}

void ActionSequencePanel::onMoveDownAction()
{
    int row = d_->action_list->currentRow();
    if (row < 0 || row >= d_->actions.size() - 1) {
        return;
    }

    d_->actions.swapItemsAt(row, row + 1);
    auto* item = d_->action_list->takeItem(row);
    d_->action_list->insertItem(row + 1, item);
    d_->action_list->setCurrentItem(item);

    Q_EMIT actionMoved(row, row + 1);
    updateButtons();
}

void ActionSequencePanel::onActionSelected(int index)
{
    if (index < 0 || index >= d_->actions.size()) {
        d_->current_action = nullptr;
        d_->configurator->setAction(nullptr);
    } else {
        d_->current_action = d_->actions[index];
        d_->configurator->setAction(d_->current_action);
    }

    updateButtons();
}

void ActionSequencePanel::onActionCompleted(ActionBlock* action)
{
    if (!d_->is_running || action != d_->current_action) {
        return;
    }

    Q_EMIT actionCompleted(action);

    d_->current_action_index++;
    if (d_->current_action_index >= d_->actions.size()) {
        d_->is_running = false;
        d_->current_action_index = -1;
        d_->current_action = nullptr;
        updateButtons();
        Q_EMIT sequenceCompleted();
    } else {
        executeNext();
    }
}

void ActionSequencePanel::onActionFailed(ActionBlock* action, const QString& error)
{
    if (!d_->is_running || action != d_->current_action) {
        return;
    }

    d_->is_running = false;
    d_->current_action_index = -1;
    d_->current_action = nullptr;
    updateButtons();

    Q_EMIT actionFailed(action, error);
    Q_EMIT sequenceFailed(error);
}

void ActionSequencePanel::connectAction(ActionBlock* action)
{
    if (!action) {
        return;
    }

    connect(action, &ActionBlock::started,
            this, [this, action]() { Q_EMIT actionStarted(action); });
    connect(action, &ActionBlock::stopped,
            this, [this, action]() { Q_EMIT actionStopped(action); });
    connect(action, &ActionBlock::paused,
            this, [this, action]() { Q_EMIT actionPaused(action); });
    connect(action, &ActionBlock::resumed,
            this, [this, action]() { Q_EMIT actionResumed(action); });
    connect(action, &ActionBlock::completed,
            this, [this, action](bool success) {
                if (success) {
                    onActionCompleted(action);
                } else {
                    onActionFailed(action, tr("动作执行失败"));
                }
            });
    connect(action, &ActionBlock::failed,
            this, [this, action](const QString& error) {
                onActionFailed(action, error);
            });
}

void ActionSequencePanel::disconnectAction(ActionBlock* action)
{
    if (!action) {
        return;
    }

    disconnect(action, nullptr, this, nullptr);
}

void ActionSequencePanel::onSequenceStarted()
{
    updateButtons();
    Q_EMIT sequenceStarted();
}

void ActionSequencePanel::onSequenceStopped()
{
    updateButtons();
    Q_EMIT sequenceStopped();
}

void ActionSequencePanel::onSequenceCompleted()
{
    updateButtons();
    Q_EMIT sequenceCompleted();
}

void ActionSequencePanel::onSequenceFailed(const QString& error)
{
    Q_EMIT sequenceFailed(error);
}

void ActionSequencePanel::clear()
{
    for (auto* action : d_->actions) {
        disconnectAction(action);
        action->deleteLater();
    }
    d_->actions.clear();
    d_->action_list->clear();
    d_->current_action_index = -1;
    d_->current_action = nullptr;
    updateButtons();
}

void ActionSequencePanel::load(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, tr("错误"), tr("无法打开文件: %1").arg(filename));
        return;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (doc.isNull()) {
        QMessageBox::warning(this, tr("错误"), tr("无效的JSON文件"));
        return;
    }

    clear();

    QJsonArray array = doc.array();
    for (const auto& value : array) {
        QJsonObject obj = value.toObject();
        QString type = obj["type"].toString();
        if (d_->factory) {
            if (auto* action = d_->factory->createAction(type)) {
                action->fromJson(obj);
                connectAction(action);
                d_->actions.append(action);
                d_->action_list->addItem(action->name());
            }
        }
    }

    updateButtons();
}

void ActionSequencePanel::save(const QString& filename)
{
    QJsonArray array;
    for (auto* action : d_->actions) {
        array.append(action->toJson());
    }

    QJsonDocument doc(array);
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, tr("错误"), tr("无法保存文件: %1").arg(filename));
        return;
    }

    file.write(doc.toJson());
}

void ActionSequencePanel::executeNext()
{
    if (!d_->is_running || d_->is_paused) {
        return;
    }

    if (d_->current_action_index >= d_->actions.size()) {
        d_->is_running = false;
        d_->current_action_index = -1;
        d_->current_action = nullptr;
        updateButtons();
        Q_EMIT sequenceCompleted();
        return;
    }

    auto* action = d_->actions[d_->current_action_index];
    d_->current_action = action;
    action->execute();
} 