#include "ui/action_list_widget.h"
#include "ui/action_block.h"
#include "ui/navigation_action_block.h"
#include "ui/loop_action_block.h"
#include "ui/condition_action_block.h"
#include "ros/robot_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolBar>
#include <QListWidget>
#include <QPushButton>
#include <QMessageBox>
#include <QIcon>

struct ActionListWidget::Private
{
    QListWidget* list_widget{nullptr};
    QPushButton* up_button{nullptr};
    QPushButton* down_button{nullptr};
    QPushButton* remove_button{nullptr};
    QPushButton* start_button{nullptr};
    QPushButton* stop_button{nullptr};
    QPushButton* pause_button{nullptr};
    ActionBlock* current_action{nullptr};
    bool is_executing{false};
    bool is_paused{false};
    std::shared_ptr<RobotController> robot_controller;
    int current_executing_index{-1};
};

ActionListWidget::ActionListWidget(QWidget* parent)
    : QWidget(parent)
    , d_(std::make_unique<Private>())
{
    setupUi();
    connectSignals();
}

ActionListWidget::~ActionListWidget() = default;

void ActionListWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // 创建工具栏
    createToolBar();

    // 创建列表部件
    d_->list_widget = new QListWidget(this);
    d_->list_widget->setDragDropMode(QAbstractItemView::InternalMove);
    d_->list_widget->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(d_->list_widget, &QListWidget::currentItemChanged,
            this, &ActionListWidget::onActionSelectionChanged);
    layout->addWidget(d_->list_widget);

    // 创建按钮布局
    auto* button_layout = new QHBoxLayout;
    button_layout->setContentsMargins(5, 5, 5, 5);
    button_layout->setSpacing(5);

    // 创建上移按钮
    d_->up_button = new QPushButton(tr("上移"), this);
    d_->up_button->setEnabled(false);
    connect(d_->up_button, &QPushButton::clicked,
            this, &ActionListWidget::onMoveActionUp);
    button_layout->addWidget(d_->up_button);

    // 创建下移按钮
    d_->down_button = new QPushButton(tr("下移"), this);
    d_->down_button->setEnabled(false);
    connect(d_->down_button, &QPushButton::clicked,
            this, &ActionListWidget::onMoveActionDown);
    button_layout->addWidget(d_->down_button);

    button_layout->addStretch();

    // 创建执行控制按钮
    d_->start_button = new QPushButton(tr("开始"), this);
    connect(d_->start_button, &QPushButton::clicked,
            this, &ActionListWidget::executeActions);
    button_layout->addWidget(d_->start_button);

    d_->pause_button = new QPushButton(tr("暂停"), this);
    d_->pause_button->setEnabled(false);
    connect(d_->pause_button, &QPushButton::clicked,
            this, &ActionListWidget::pauseActions);
    button_layout->addWidget(d_->pause_button);

    d_->stop_button = new QPushButton(tr("停止"), this);
    d_->stop_button->setEnabled(false);
    connect(d_->stop_button, &QPushButton::clicked,
            this, &ActionListWidget::stopActions);
    button_layout->addWidget(d_->stop_button);

    // 创建删除按钮
    d_->remove_button = new QPushButton(tr("删除"), this);
    d_->remove_button->setEnabled(false);
    connect(d_->remove_button, &QPushButton::clicked,
            this, &ActionListWidget::onRemoveAction);
    button_layout->addWidget(d_->remove_button);

    layout->addLayout(button_layout);
}

void ActionListWidget::createToolBar()
{
    auto* toolbar = new QToolBar(this);
    toolbar->setIconSize(QSize(32, 32));
    toolbar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    // 添加导航动作按钮
    auto* nav_action = toolbar->addAction(QIcon::fromTheme("go-next"), tr("导航"));
    connect(nav_action, &QAction::triggered,
            this, &ActionListWidget::onAddNavigationAction);

    // 添加循环动作按钮
    auto* loop_action = toolbar->addAction(QIcon::fromTheme("view-refresh"), tr("循环"));
    connect(loop_action, &QAction::triggered,
            this, &ActionListWidget::onAddLoopAction);

    // 添加条件动作按钮
    auto* condition_action = toolbar->addAction(QIcon::fromTheme("dialog-question"), tr("条件"));
    connect(condition_action, &QAction::triggered,
            this, &ActionListWidget::onAddConditionAction);

    layout()->addWidget(toolbar);
}

void ActionListWidget::connectSignals()
{
    connect(d_->list_widget, &QListWidget::itemSelectionChanged,
            this, &ActionListWidget::updateButtons);
}

void ActionListWidget::setRobotController(const std::shared_ptr<RobotController>& controller)
{
    d_->robot_controller = controller;
}

void ActionListWidget::onAddNavigationAction()
{
    if (!d_->robot_controller) {
        QMessageBox::warning(this, tr("错误"), tr("机器人控制器未初始化"));
        return;
    }
    auto* action = new NavigationActionBlock(d_->robot_controller);
    auto* item = new QListWidgetItem(tr("导航动作"));
    item->setData(Qt::UserRole, QVariant::fromValue(action));
    d_->list_widget->addItem(item);
    d_->list_widget->setCurrentItem(item);
}

void ActionListWidget::onAddLoopAction()
{
    auto* action = new LoopActionBlock;
    auto* item = new QListWidgetItem(tr("循环动作"));
    item->setData(Qt::UserRole, QVariant::fromValue(action));
    d_->list_widget->addItem(item);
    d_->list_widget->setCurrentItem(item);
}

void ActionListWidget::onAddConditionAction()
{
    if (!d_->robot_controller) {
        QMessageBox::warning(this, tr("错误"), tr("机器人控制器未初始化"));
        return;
    }
    auto* action = new ConditionActionBlock(d_->robot_controller);
    auto* item = new QListWidgetItem(tr("条件动作"));
    item->setData(Qt::UserRole, QVariant::fromValue(action));
    d_->list_widget->addItem(item);
    d_->list_widget->setCurrentItem(item);
}

void ActionListWidget::onRemoveAction()
{
    auto* item = d_->list_widget->currentItem();
    if (!item) return;

    auto* action = item->data(Qt::UserRole).value<ActionBlock*>();
    delete action;
    delete item;

    updateButtons();
}

void ActionListWidget::onMoveActionUp()
{
    int row = d_->list_widget->currentRow();
    if (row <= 0) return;

    auto* item = d_->list_widget->takeItem(row);
    d_->list_widget->insertItem(row - 1, item);
    d_->list_widget->setCurrentItem(item);
}

void ActionListWidget::onMoveActionDown()
{
    int row = d_->list_widget->currentRow();
    if (row >= d_->list_widget->count() - 1) return;

    auto* item = d_->list_widget->takeItem(row);
    d_->list_widget->insertItem(row + 1, item);
    d_->list_widget->setCurrentItem(item);
}

void ActionListWidget::onActionSelectionChanged()
{
    QList<QListWidgetItem*> selected_items = d_->list_widget->selectedItems();
    if (selected_items.isEmpty()) {
        d_->current_action = nullptr;
        Q_EMIT actionSelected(nullptr);
        return;
    }

    auto* item = selected_items.first();
    d_->current_action = item->data(Qt::UserRole).value<ActionBlock*>();
    Q_EMIT actionSelected(d_->current_action);
}

void ActionListWidget::updateButtons()
{
    bool has_selection = d_->list_widget->currentItem() != nullptr;
    int current_row = d_->list_widget->currentRow();
    int count = d_->list_widget->count();

    d_->up_button->setEnabled(!d_->is_executing && has_selection && current_row > 0);
    d_->down_button->setEnabled(!d_->is_executing && has_selection && current_row < count - 1);
    d_->remove_button->setEnabled(!d_->is_executing && has_selection);
    
    d_->start_button->setEnabled(!d_->is_executing && count > 0);
    d_->pause_button->setEnabled(d_->is_executing && !d_->is_paused);
    d_->stop_button->setEnabled(d_->is_executing);
}

bool ActionListWidget::validateActions() const
{
    for (int i = 0; i < d_->list_widget->count(); ++i) {
        auto* item = d_->list_widget->item(i);
        auto* action = item->data(Qt::UserRole).value<ActionBlock*>();
        if (!action->isValid()) {
            return false;
        }
    }
    return true;
}

void ActionListWidget::executeActions()
{
    if (d_->is_executing) {
        return;
    }

    if (d_->list_widget->count() == 0) {
        QMessageBox::warning(this, tr("警告"), tr("没有可执行的动作"));
        return;
    }

    if (!validateActions()) {
        QMessageBox::warning(this, tr("警告"), tr("存在无效的动作，请检查设置"));
        return;
    }

    d_->is_executing = true;
    d_->is_paused = false;
    d_->current_executing_index = 0;
    updateButtons();
    Q_EMIT executionStarted();
    
    executeNextAction();
}

void ActionListWidget::stopActions()
{
    if (!d_->is_executing) {
        return;
    }

    if (auto* item = d_->list_widget->item(d_->current_executing_index)) {
        if (auto* action = item->data(Qt::UserRole).value<ActionBlock*>()) {
            action->stop();
        }
    }

    d_->is_executing = false;
    d_->is_paused = false;
    d_->current_executing_index = -1;
    updateButtons();
    Q_EMIT executionStopped();
}

void ActionListWidget::pauseActions()
{
    if (!d_->is_executing || d_->is_paused) {
        return;
    }

    if (auto* item = d_->list_widget->item(d_->current_executing_index)) {
        if (auto* action = item->data(Qt::UserRole).value<ActionBlock*>()) {
            action->pause();
        }
    }

    d_->is_paused = true;
    updateButtons();
    Q_EMIT executionPaused();
}

void ActionListWidget::resumeActions()
{
    if (!d_->is_executing || !d_->is_paused) {
        return;
    }

    if (auto* item = d_->list_widget->item(d_->current_executing_index)) {
        if (auto* action = item->data(Qt::UserRole).value<ActionBlock*>()) {
            action->resume();
        }
    }

    d_->is_paused = false;
    updateButtons();
    Q_EMIT executionResumed();
}

void ActionListWidget::executeNextAction()
{
    if (!d_->is_executing || d_->is_paused) {
        return;
    }

    if (d_->current_executing_index >= d_->list_widget->count()) {
        stopActions();
        return;
    }

    auto* item = d_->list_widget->item(d_->current_executing_index);
    if (!item) {
        stopActions();
        return;
    }

    auto* action = item->data(Qt::UserRole).value<ActionBlock*>();
    if (!action) {
        stopActions();
        return;
    }

    connect(action, &ActionBlock::completed,
            this, &ActionListWidget::onActionCompleted);
    connect(action, &ActionBlock::error,
            this, &ActionListWidget::onActionError);
    connect(action, &ActionBlock::progressChanged,
            this, &ActionListWidget::onActionProgressChanged);

    action->execute();
}

void ActionListWidget::onActionCompleted()
{
    if (!d_->is_executing || d_->is_paused) {
        return;
    }

    auto* action = qobject_cast<ActionBlock*>(sender());
    if (!action) {
        return;
    }

    disconnect(action, &ActionBlock::completed,
              this, &ActionListWidget::onActionCompleted);
    disconnect(action, &ActionBlock::error,
              this, &ActionListWidget::onActionError);
    disconnect(action, &ActionBlock::progressChanged,
              this, &ActionListWidget::onActionProgressChanged);

    d_->current_executing_index++;
    executeNextAction();
}

void ActionListWidget::onActionError(const QString& message)
{
    QMessageBox::critical(this, tr("错误"), message);
    stopActions();
}

void ActionListWidget::onActionProgressChanged(double progress)
{
    Q_EMIT executionProgressChanged(d_->current_executing_index, progress);
} 