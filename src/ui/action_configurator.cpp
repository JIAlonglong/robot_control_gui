/**
 * Copyright (c) 2024 JIAlonglong
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
 * @file action_configurator.cpp
 * @brief 动作配置器界面类的实现
 * @author JIAlonglong
 */

#include "action_configurator.h"
#include <QAction>
#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTextBrowser>
#include <QToolBar>
#include <QUndoCommand>
#include <QUndoStack>
#include <QUndoView>
#include <QVBoxLayout>
#include <stdexcept>
#include "action_block_factory.h"
#include "action_block_widget.h"
#include "action_commands.h"
#include "action_selection_dialog.h"
#include "action_sequence_manager.h"
#include "connection_line.h"
#include "path_editor.h"
#include "path_visualizer.h"
#include "property_editor.h"

ActionConfigurator::ActionConfigurator(const std::shared_ptr<ActionBlockFactory>& factory,
                                       QWidget*                                   parent)
    : QWidget(parent)
{
    factory_         = factory;
    manager_         = new ActionSequenceManager();
    undo_stack_      = new QUndoStack(this);
    property_editor_ = new PropertyEditor(this);
    setupUi();
    connectSignals();
}

ActionConfigurator::~ActionConfigurator()
{
    delete manager_;
}

void ActionConfigurator::setupUi()
{
    main_layout_ = new QVBoxLayout(this);

    // 创建工具栏
    auto* toolbar = new QToolBar(this);
    toolbar->setIconSize(QSize(32, 32));

    // 添加动作按钮
    add_action_button_     = new QPushButton(tr("添加动作"), this);
    remove_action_button_  = new QPushButton(tr("删除动作"), this);
    start_sequence_button_ = new QPushButton(tr("开始"), this);
    stop_sequence_button_  = new QPushButton(tr("停止"), this);

    toolbar->addWidget(add_action_button_);
    toolbar->addWidget(remove_action_button_);
    toolbar->addSeparator();
    toolbar->addWidget(start_sequence_button_);
    toolbar->addWidget(stop_sequence_button_);

    main_layout_->addWidget(toolbar);

    // 创建动作类型选择下拉框
    action_type_combo_ = new QComboBox(this);
    if (factory_) {
        for (const auto& type : factory_->availableTypes()) {
            action_type_combo_->addItem(type);
        }
    }

    // 创建属性编辑器
    property_editor_ = new PropertyEditor(this);

    // 创建撤销栈和视图
    undo_stack_     = new QUndoStack(this);
    auto* undo_view = new QUndoView(undo_stack_, this);

    // 添加到布局
    auto* hsplitter = new QSplitter(Qt::Horizontal, this);
    hsplitter->addWidget(undo_view);
    hsplitter->addWidget(property_editor_);
    main_layout_->addWidget(hsplitter);
}

void ActionConfigurator::connectSignals()
{
    connect(add_action_button_, &QPushButton::clicked, this,
            &ActionConfigurator::onAddActionClicked);
    connect(remove_action_button_, &QPushButton::clicked, this,
            &ActionConfigurator::onRemoveAction);
    connect(start_sequence_button_, &QPushButton::clicked, this,
            &ActionConfigurator::onStartSequence);
    connect(stop_sequence_button_, &QPushButton::clicked, this,
            &ActionConfigurator::onStopSequence);

    if (property_editor_) {
        connect(property_editor_, &PropertyEditor::propertyChanged, this,
                &ActionConfigurator::onPropertyChanged);
    }
}

void ActionConfigurator::onAddActionClicked()
{
    if (!factory_) {
        return;
    }

    QString type = action_type_combo_->currentText();
    if (type.isEmpty()) {
        return;
    }

    auto* action = factory_->create(type, this);
    if (action) {
        if (manager_) {
            manager_->addAction(action);
        }
        setCurrentAction(action);
        Q_EMIT actionAdded(action);
    }
}

void ActionConfigurator::onRemoveAction()
{
    if (current_action_ && manager_) {
        manager_->removeAction(current_action_);
        Q_EMIT actionRemoved(current_action_);
        setCurrentAction(nullptr);
    }
}

void ActionConfigurator::onStartSequence()
{
    if (manager_) {
        manager_->start();
        Q_EMIT sequenceStarted();
    }
}

void ActionConfigurator::onStopSequence()
{
    if (manager_) {
        manager_->stop();
        Q_EMIT sequenceStopped();
    }
}

void ActionConfigurator::onLoadConfig()
{
    QString fileName =
        QFileDialog::getOpenFileName(this, tr("加载配置"), QString(), tr("JSON文件 (*.json)"));
    if (!fileName.isEmpty()) {
        loadConfig(fileName);
    }
}

void ActionConfigurator::onSaveConfig()
{
    QString fileName =
        QFileDialog::getSaveFileName(this, tr("保存配置"), QString(), tr("JSON文件 (*.json)"));
    if (!fileName.isEmpty()) {
        saveConfig(fileName);
    }
}

void ActionConfigurator::onHelpRequested()
{
    QMessageBox::information(this, tr("帮助"),
                             tr("这是一个用于配置机器人动作序列的工具。\n\n"
                                "1. 使用\"添加动作\"按钮添加新的动作\n"
                                "2. 设置动作的参数\n"
                                "3. 使用工具栏按钮来控制序列的执行\n"
                                "4. 可以保存和加载配置文件"));
}

void ActionConfigurator::onPropertyChanged(const QString& name, const QVariant& value)
{
    if (current_action_) {
        current_action_->setProperty(name, value);
        Q_EMIT actionConfigured(current_action_);
    }
}

void ActionConfigurator::onBlockMoved(ActionBlockWidget* widget, const QPointF& oldPos,
                                      const QPointF& newPos)
{
    // TODO: 实现动作块移动的撤销/重做
}

void ActionConfigurator::onConnectionStarted(ActionBlockWidget* source, const QPointF& pos)
{
    // TODO: 实现连接开始的逻辑
}

void ActionConfigurator::onConnectionFinished(ActionBlockWidget* target, const QPointF& pos)
{
    // TODO: 实现连接完成的逻辑
}

void ActionConfigurator::onWaypointMoved(int index, double x, double y, double theta)
{
    // TODO: 实现路径点移动的逻辑
}

void ActionConfigurator::setCurrentAction(ActionBlock* action)
{
    if (current_action_ != action) {
        current_action_ = action;
        if (property_editor_) {
            property_editor_->setActionBlock(action);
        }
        Q_EMIT currentActionChanged(action);
    }
}

void ActionConfigurator::loadConfig(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        Q_EMIT configurationError(tr("无法打开文件: %1").arg(filename));
        return;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (doc.isNull()) {
        Q_EMIT configurationError(tr("无效的JSON文件"));
        return;
    }

    loadFromJson(doc.object());
}

void ActionConfigurator::saveConfig(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        Q_EMIT configurationError(tr("无法保存文件: %1").arg(filename));
        return;
    }

    QJsonDocument doc(saveToJson());
    file.write(doc.toJson());
}

QJsonObject ActionConfigurator::saveToJson() const
{
    QJsonObject obj;
    // TODO: 实现配置序列化
    return obj;
}

void ActionConfigurator::loadFromJson(const QJsonObject& obj)
{
    // TODO: 实现配置反序列化
}

void ActionConfigurator::setActionBlockFactory(ActionBlockFactory* factory)
{
    factory_ = std::shared_ptr<ActionBlockFactory>(factory);
    if (factory_ && action_type_combo_) {
        action_type_combo_->clear();
        for (const auto& type : factory_->availableTypes()) {
            action_type_combo_->addItem(type);
        }
    }
}

void ActionConfigurator::setActionSequenceManager(ActionSequenceManager* manager)
{
    manager_ = manager;
}

void ActionConfigurator::clear()
{
    if (manager_) {
        manager_->clearActions();
    }
    setCurrentAction(nullptr);
    if (undo_stack_) {
        undo_stack_->clear();
    }
}

#include "action_configurator.moc"