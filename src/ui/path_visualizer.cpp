#include "ui/path_visualizer.h"
#include "ui/path_action_block.h"
#include <QPainter>
#include <QWheelEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsPixmapItem>
#include <QStyleOptionGraphicsItem>
#include <cmath>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QScrollBar>

// WaypointItem 实现
WaypointItem::WaypointItem(int index, QGraphicsItem* parent)
    : QGraphicsItem(parent)
    , index_(index)
{
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    setAcceptHoverEvents(true);
}

QRectF WaypointItem::boundingRect() const
{
    return QRectF(-15, -15, 30, 30);
}

void WaypointItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(option);

    // 简化绘制，只保留基本形状
    painter->setPen(Qt::black);
    painter->setBrush(highlighted_ ? Qt::red : Qt::green);
    painter->drawEllipse(-10, -10, 20, 20);
    
    // 绘制方向箭头
    painter->drawLine(0, 0, 15, 0);
    painter->drawLine(15, 0, 10, -5);
    painter->drawLine(15, 0, 10, 5);

    // 绘制索引
    painter->drawText(boundingRect(), Qt::AlignCenter, QString::number(index_ + 1));
}

void WaypointItem::setHighlighted(bool highlighted)
{
    if (highlighted_ != highlighted) {
        highlighted_ = highlighted;
        update();
    }
}

void WaypointItem::setPosition(const QPointF& pos)
{
    setPos(pos);
}

void WaypointItem::setRotation(double theta)
{
    QGraphicsItem::setRotation(theta * 180.0 / M_PI);
}

void WaypointItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        drag_start_ = event->pos();
        is_dragging_ = true;
    }
    QGraphicsItem::mousePressEvent(event);
}

void WaypointItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    if (is_dragging_) {
        QPointF delta = event->pos() - drag_start_;
        setPos(pos() + delta);
        if (auto* visualizer = qobject_cast<PathVisualizer*>(scene()->views().first())) {
            visualizer->onWaypointMoved(this);
        }
    }
    QGraphicsItem::mouseMoveEvent(event);
}

void WaypointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = false;
    }
    QGraphicsItem::mouseReleaseEvent(event);
}

void WaypointItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    hover_ = true;
    update();
    QGraphicsItem::hoverEnterEvent(event);
}

void WaypointItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    hover_ = false;
    update();
    QGraphicsItem::hoverLeaveEvent(event);
}

// PathVisualizer 实现
PathVisualizer::PathVisualizer(QWidget* parent)
    : QGraphicsView(parent)
    , scene_(new QGraphicsScene(this))
    , current_path_(nullptr)
{
    setScene(scene_);
    setRenderHint(QPainter::Antialiasing);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
}

PathVisualizer::~PathVisualizer()
{
    clear();
}

void PathVisualizer::setPathAction(PathActionBlock* action)
{
    if (current_path_ == action) {
        return;
    }

    current_path_ = action;
    updatePath();
}

void PathVisualizer::clear()
{
    current_path_ = nullptr;
    qDeleteAll(waypoint_items_);
    waypoint_items_.clear();
    qDeleteAll(path_items_);
    path_items_.clear();
    delete preview_line_;
    preview_line_ = nullptr;
}

void PathVisualizer::setMapImage(const QImage& image)
{
    if (!scene_) {
        return;
    }

    if (!map_item_) {
        map_item_ = new QGraphicsPixmapItem();
        scene_->addItem(map_item_);
        map_item_->setZValue(-1);
    }

    map_item_->setPixmap(QPixmap::fromImage(image));
    map_item_->setPos(-image.width() / 2, -image.height() / 2);
    resetView();
}

void PathVisualizer::updatePath()
{
    scene_->clear();
    waypoint_items_.clear();

    if (!current_path_) {
        return;
    }

    QJsonArray waypoints = current_path_->waypoints();
    if (waypoints.isEmpty()) {
        return;
    }

    // 绘制路径线
    QPainterPath path;
    bool first = true;
    for (const QJsonValue& value : waypoints) {
        QJsonObject waypoint = value.toObject();
        double x = waypoint["x"].toDouble();
        double y = waypoint["y"].toDouble();
        
        if (first) {
            path.moveTo(x, y);
            first = false;
        } else {
            path.lineTo(x, y);
        }

        // 添加路径点标记
        auto* ellipse = new QGraphicsEllipseItem(x - 5, y - 5, 10, 10);
        ellipse->setBrush(QBrush(Qt::blue));
        ellipse->setPen(QPen(Qt::blue, 1));
        ellipse->setFlag(QGraphicsItem::ItemIsMovable);
        ellipse->setFlag(QGraphicsItem::ItemIsSelectable);
        
        scene_->addItem(ellipse);
        waypoint_items_.append(ellipse);
    }

    // 添加路径线
    auto* path_item = scene_->addPath(path, QPen(Qt::blue, 2));
    path_item->setZValue(-1);

    // 调整视图以显示整个路径
    fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void PathVisualizer::highlightWaypoint(int index)
{
    // 高亮显示选中的路径点
    for (auto* item : waypoint_items_) {
        auto* ellipse = qgraphicsitem_cast<QGraphicsEllipseItem*>(item);
        if (ellipse) {
            int item_index = waypoint_items_.indexOf(item);
            if (item_index == index) {
                ellipse->setBrush(QBrush(Qt::red));
                ellipse->setPen(QPen(Qt::red, 2));
            } else {
                ellipse->setBrush(QBrush(Qt::blue));
                ellipse->setPen(QPen(Qt::blue, 1));
            }
        }
    }
}

void PathVisualizer::onWaypointMoved(WaypointItem* item)
{
    if (!item || !current_path_) {
        return;
    }

    QPointF pos = sceneToMap(item->pos());
    double theta = item->rotation() * M_PI / 180.0;
    emit waypointMoved(waypoint_items_.indexOf(item), pos.x(), pos.y(), theta);
}

void PathVisualizer::createPathItems()
{
    qDeleteAll(path_items_);
    path_items_.clear();

    if (!current_path_) {
        return;
    }

    QJsonArray waypoints = current_path_->waypoints();
    for (int i = 0; i < waypoints.size() - 1; ++i) {
        auto* line = new QGraphicsLineItem();
        line->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        scene_->addItem(line);
        path_items_.append(line);
    }

    updatePathItems();
}

void PathVisualizer::updatePathItems()
{
    if (path_items_.isEmpty()) {
        createPathItems();
    }

    if (!current_path_) {
        return;
    }

    QJsonArray waypoints = current_path_->waypoints();
    for (int i = 0; i < path_items_.size(); ++i) {
        QJsonObject start = waypoints[i].toObject();
        QJsonObject end = waypoints[i + 1].toObject();
        
        QPointF start_pos = mapToScene(start["x"].toDouble(), start["y"].toDouble());
        QPointF end_pos = mapToScene(end["x"].toDouble(), end["y"].toDouble());
        
        if (auto* line = qgraphicsitem_cast<QGraphicsLineItem*>(path_items_[i])) {
            line->setLine(QLineF(start_pos, end_pos));
        }
    }
}

QPointF PathVisualizer::mapToScene(double x, double y) const
{
    return QPointF((x - map_origin_x_) / map_resolution_,
                  (y - map_origin_y_) / map_resolution_);
}

QPointF PathVisualizer::sceneToMap(const QPointF& pos) const
{
    return QPointF(pos.x() * map_resolution_ + map_origin_x_,
                  pos.y() * map_resolution_ + map_origin_y_);
}

void PathVisualizer::wheelEvent(QWheelEvent* event)
{
    double factor = std::pow(1.2, event->angleDelta().y() / 240.0);
    QGraphicsView::scale(factor, factor);
}

void PathVisualizer::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton) {
        is_panning_ = true;
        last_mouse_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QGraphicsView::mousePressEvent(event);
}

void PathVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    if (is_panning_) {
        QPointF delta = event->pos() - last_mouse_pos_;
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
        verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
        last_mouse_pos_ = event->pos();
        event->accept();
        return;
    }
    QGraphicsView::mouseMoveEvent(event);
}

void PathVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton) {
        is_panning_ = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void PathVisualizer::zoomIn()
{
    scale(1.2, 1.2);
}

void PathVisualizer::zoomOut()
{
    scale(1.0 / 1.2, 1.0 / 1.2);
}

void PathVisualizer::resetView()
{
    if (!scene_) {
        return;
    }

    // 重置变换
    resetTransform();

    // 适应视图
    fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);

    // 居中显示
    centerOn(scene_->sceneRect().center());
}

#include "path_visualizer.moc" 