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
 * @file path_editor.cpp
 * @brief 路径编辑器组件的实现,用于编辑机器人的导航路径
 * @author JIAlonglong
 */

#include "path_editor.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidgetItem>
#include <QVBoxLayout>

PathEditor::PathEditor(QWidget* parent)
    : QWidget(parent)
    , main_layout_(new QVBoxLayout(this))
    , toolbar_(new QToolBar(this))
    , table_(new QTableWidget(this))
    , path_action_(nullptr)
    , undo_stack_(new QUndoStack(this))
    , scene_(new QGraphicsScene(this))
    , view_(new QGraphicsView(scene_, this))
    , x_spin_(new QSpinBox(this))
    , y_spin_(new QSpinBox(this))
{
    setupUI();
    connectSignals();
}

void PathEditor::setupUI()
{
    // 创建工具栏
    createToolBar();
    main_layout_->addWidget(toolbar_);

    // 创建表格
    createTable();
    main_layout_->addWidget(table_);

    // 创建图形视图
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    main_layout_->addWidget(view_);

    // 创建坐标输入控件
    auto* coord_layout = new QHBoxLayout;
    coord_layout->addWidget(new QLabel("X:", this));
    x_spin_->setRange(-10000, 10000);
    coord_layout->addWidget(x_spin_);
    coord_layout->addWidget(new QLabel("Y:", this));
    y_spin_->setRange(-10000, 10000);
    coord_layout->addWidget(y_spin_);
    coord_layout->addStretch();
    main_layout_->addLayout(coord_layout);
}

void PathEditor::createToolBar()
{
    auto* add_btn = new QPushButton(tr("添加路径点"), this);
    connect(add_btn, &QPushButton::clicked, this, &PathEditor::addWaypoint);
    toolbar_->addWidget(add_btn);

    auto* remove_btn = new QPushButton(tr("删除路径点"), this);
    connect(remove_btn, &QPushButton::clicked, this, &PathEditor::removeWaypoint);
    toolbar_->addWidget(remove_btn);

    auto* up_btn = new QPushButton(tr("上移"), this);
    connect(up_btn, &QPushButton::clicked, this, &PathEditor::moveWaypointUp);
    toolbar_->addWidget(up_btn);

    auto* down_btn = new QPushButton(tr("下移"), this);
    connect(down_btn, &QPushButton::clicked, this, &PathEditor::moveWaypointDown);
    toolbar_->addWidget(down_btn);

    auto* clear_btn = new QPushButton(tr("清空"), this);
    connect(clear_btn, &QPushButton::clicked, this, &PathEditor::clearWaypoints);
    toolbar_->addWidget(clear_btn);
}

void PathEditor::createTable()
{
    table_->setColumnCount(3);
    table_->setHorizontalHeaderLabels({tr("X"), tr("Y"), tr("角度")});
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(table_, &QTableWidget::itemChanged, this, &PathEditor::onItemChanged);
    connect(table_, &QTableWidget::currentCellChanged, this,
            [this](int row, int, int, int) { onWaypointSelected(row); });
    connect(table_, &QTableWidget::cellDoubleClicked, this, &PathEditor::onWaypointDoubleClicked);
}

void PathEditor::connectSignals()
{
    connect(x_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &PathEditor::onSpinValueChanged);
    connect(y_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &PathEditor::onSpinValueChanged);
}

void PathEditor::setPathAction(PathActionBlock* action)
{
    if (path_action_ == action) {
        return;
    }

    path_action_ = action;
    updateTable();
}

void PathEditor::clear()
{
    table_->clearContents();
    table_->setRowCount(0);
    scene_->clear();
}

void PathEditor::addWaypoint()
{
    if (!path_action_) {
        return;
    }

    path_action_->addWaypoint(0, 0, 0);
    updateTable();
}

void PathEditor::removeWaypoint()
{
    if (!path_action_) {
        return;
    }

    int row = table_->currentRow();
    if (row >= 0) {
        path_action_->removeWaypoint(row);
        updateTable();
    }
}

void PathEditor::moveWaypointUp()
{
    if (!path_action_) {
        return;
    }

    int row = table_->currentRow();
    if (row > 0) {
        path_action_->moveWaypoint(row, row - 1);
        updateTable();
        table_->selectRow(row - 1);
    }
}

void PathEditor::moveWaypointDown()
{
    if (!path_action_) {
        return;
    }

    int row = table_->currentRow();
    if (row >= 0 && row < table_->rowCount() - 1) {
        path_action_->moveWaypoint(row, row + 1);
        updateTable();
        table_->selectRow(row + 1);
    }
}

void PathEditor::clearWaypoints()
{
    if (!path_action_) {
        return;
    }

    path_action_->clearWaypoints();
    updateTable();
}

void PathEditor::updateWaypoint(int row, int column)
{
    if (!path_action_) {
        return;
    }

    QJsonArray waypoints = path_action_->waypoints();
    if (row >= 0 && row < waypoints.size()) {
        QJsonObject       waypoint = waypoints[row].toObject();
        QTableWidgetItem* item     = table_->item(row, column);
        if (!item) {
            return;
        }

        bool   ok;
        double value = item->text().toDouble(&ok);
        if (!ok) {
            return;
        }

        switch (column) {
            case 0:
                waypoint["x"] = value;
                break;
            case 1:
                waypoint["y"] = value;
                break;
            case 2:
                waypoint["theta"] = value;
                break;
        }

        waypoints[row] = waypoint;
        path_action_->setWaypoints(waypoints);
        emit waypointModified(row);
    }
}

void PathEditor::onWaypointSelected(int row)
{
    emit waypointSelected(row);
}

void PathEditor::onWaypointDoubleClicked(int row, int column)
{
    if (row >= 0 && column >= 0) {
        table_->editItem(table_->item(row, column));
    }
}

void PathEditor::undo()
{
    undo_stack_->undo();
}

void PathEditor::redo()
{
    undo_stack_->redo();
}

void PathEditor::onItemChanged(QTableWidgetItem* item)
{
    if (!item || !path_action_) {
        return;
    }

    updateWaypoint(item->row(), item->column());
}

void PathEditor::onAddPoint()
{
    addWaypoint();
}

void PathEditor::onRemovePoint()
{
    removeWaypoint();
}

void PathEditor::onSpinValueChanged()
{
    int row = table_->currentRow();
    if (row >= 0) {
        QJsonArray waypoints = path_action_->waypoints();
        if (row < waypoints.size()) {
            QJsonObject waypoint = waypoints[row].toObject();
            waypoint["x"]        = x_spin_->value();
            waypoint["y"]        = y_spin_->value();
            waypoints[row]       = waypoint;
            path_action_->setWaypoints(waypoints);
            updateTable();
            emit waypointModified(row);
        }
    }
}

void PathEditor::onSelectionChanged()
{
    int row = table_->currentRow();
    if (row >= 0) {
        QJsonArray waypoints = path_action_->waypoints();
        if (row < waypoints.size()) {
            QJsonObject waypoint = waypoints[row].toObject();
            x_spin_->setValue(waypoint["x"].toInt());
            y_spin_->setValue(waypoint["y"].toInt());
        }
    }
}

void PathEditor::updateTable()
{
    if (!path_action_) {
        clear();
        return;
    }

    QJsonArray waypoints = path_action_->waypoints();
    table_->setRowCount(waypoints.size());

    for (int i = 0; i < waypoints.size(); ++i) {
        QJsonObject waypoint = waypoints[i].toObject();

        auto* x_item     = new QTableWidgetItem(QString::number(waypoint["x"].toDouble()));
        auto* y_item     = new QTableWidgetItem(QString::number(waypoint["y"].toDouble()));
        auto* theta_item = new QTableWidgetItem(QString::number(waypoint["theta"].toDouble()));

        table_->setItem(i, 0, x_item);
        table_->setItem(i, 1, y_item);
        table_->setItem(i, 2, theta_item);
    }

    // 更新图形视图
    scene_->clear();
    QPainterPath path;
    bool         first = true;

    for (const QJsonValue& value : waypoints) {
        QJsonObject waypoint = value.toObject();
        double      x        = waypoint["x"].toDouble();
        double      y        = waypoint["y"].toDouble();

        if (first) {
            path.moveTo(x, y);
            first = false;
        } else {
            path.lineTo(x, y);
        }

        QGraphicsEllipseItem* point = scene_->addEllipse(x - 5, y - 5, 10, 10);
        point->setBrush(Qt::blue);
        point->setFlag(QGraphicsItem::ItemIsMovable);
        point->setFlag(QGraphicsItem::ItemIsSelectable);
    }

    if (!waypoints.isEmpty()) {
        scene_->addPath(path, QPen(Qt::blue, 2));
        view_->fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio);
    }
}

void PathEditor::addPoint(const QPoint& point)
{
    if (!path_action_) {
        return;
    }

    path_action_->addWaypoint(point.x(), point.y(), 0);
    updateTable();
}

#include "path_editor.moc"