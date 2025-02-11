#ifndef PATH_EDITOR_H
#define PATH_EDITOR_H

#include <QWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolBar>
#include <QJsonArray>
#include <QUndoStack>
#include <QSpinBox>
#include <QGraphicsScene>
#include <QGraphicsView>
#include "ui/path_action_block.h"

class QTableWidgetItem;

class PathEditor : public QWidget
{
    Q_OBJECT

public:
    explicit PathEditor(QWidget* parent = nullptr);
    ~PathEditor() override = default;

    void setPathAction(PathActionBlock* action);
    void clear();

public Q_SLOTS:
    void addWaypoint();
    void removeWaypoint();
    void moveWaypointUp();
    void moveWaypointDown();
    void clearWaypoints();
    void updateWaypoint(int row, int column);
    void onWaypointSelected(int row);
    void onWaypointDoubleClicked(int row, int column);
    void undo();
    void redo();
    void onItemChanged(QTableWidgetItem* item);
    void onAddPoint();
    void onRemovePoint();
    void onSpinValueChanged();
    void onSelectionChanged();

Q_SIGNALS:
    void waypointSelected(int index);
    void waypointModified(int index);
    void waypointMoved(int index, double x, double y, double theta);
    void pathChanged(const QJsonArray& path);

protected:
    void setupUI();
    void createToolBar();
    void createTable();
    void updateTable();
    void connectSignals();
    void addPoint(const QPoint& point);

private:
    QVBoxLayout* main_layout_{nullptr};
    QToolBar* toolbar_{nullptr};
    QTableWidget* table_{nullptr};
    PathActionBlock* path_action_{nullptr};
    QUndoStack* undo_stack_{nullptr};
    QGraphicsScene* scene_{nullptr};
    QGraphicsView* view_{nullptr};
    QSpinBox* x_spin_{nullptr};
    QSpinBox* y_spin_{nullptr};
    QJsonArray* path_{nullptr};
};

#endif // PATH_EDITOR_H 