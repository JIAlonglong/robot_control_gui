#ifndef PATH_VISUALIZER_H
#define PATH_VISUALIZER_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QGraphicsItem>
#include <QList>
#include <QPointF>
#include <QJsonArray>

class PathActionBlock;
class WaypointItem;
class QGraphicsPixmapItem;

class PathVisualizer : public QGraphicsView
{
    Q_OBJECT

public:
    explicit PathVisualizer(QWidget* parent = nullptr);
    ~PathVisualizer() override;

    void setPathAction(PathActionBlock* action);
    void clear();
    void setMapImage(const QImage& image);
    void highlightWaypoint(int index);
    void onWaypointMoved(WaypointItem* item);

Q_SIGNALS:
    void waypointMoved(int index, double x, double y, double theta);

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void updatePath();
    void createPathItems();
    void updatePathItems();
    QPointF mapToScene(double x, double y) const;
    QPointF sceneToMap(const QPointF& pos) const;
    void zoomIn();
    void zoomOut();
    void resetView();

    QGraphicsScene* scene_;
    PathActionBlock* current_path_;
    QList<QGraphicsItem*> waypoint_items_;
    QList<QGraphicsItem*> path_items_;
    QGraphicsLineItem* preview_line_{nullptr};
    QGraphicsPixmapItem* map_item_{nullptr};
    bool is_panning_{false};
    QPointF last_mouse_pos_;
    double map_resolution_{0.05};
    double map_origin_x_{0.0};
    double map_origin_y_{0.0};
};

class WaypointItem : public QGraphicsItem
{
public:
    explicit WaypointItem(int index, QGraphicsItem* parent = nullptr);

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    void setHighlighted(bool highlighted);
    void setPosition(const QPointF& pos);
    void setRotation(double theta);
    int index() const { return index_; }

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
    int index_;
    bool highlighted_{false};
    bool hover_{false};
    QPointF drag_start_;
    bool is_dragging_{false};
};

#endif // PATH_VISUALIZER_H 