/**
 * @file map_view_widget.h
 * @brief SLAM地图显示组件
 */

#ifndef MAP_VIEW_WIDGET_H
#define MAP_VIEW_WIDGET_H

#include <QWidget>
#include <QImage>
#include <QTransform>

/**
 * @brief SLAM地图显示组件类
 * 
 * 用于显示SLAM建图结果，支持缩放、平移等操作
 */
class MapViewWidget : public QWidget {
    Q_OBJECT

public:
    explicit MapViewWidget(QWidget* parent = nullptr);

public slots:
    /**
     * @brief 更新地图数据
     * @param map_data 地图图像数据
     * @param resolution 地图分辨率(米/像素)
     */
    void updateMap(const QImage& map_data, double resolution);

    /**
     * @brief 设置机器人位置
     * @param x 机器人X坐标(米)
     * @param y 机器人Y坐标(米)
     * @param theta 机器人朝向(弧度)
     */
    void setRobotPose(double x, double y, double theta);

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QImage map_;           ///< 地图图像
    double resolution_;    ///< 地图分辨率
    QTransform transform_; ///< 视图变换矩阵
    QPoint last_mouse_pos_;///< 上次鼠标位置
    bool is_panning_;      ///< 是否正在平移
    
    struct {
        double x;
        double y;
        double theta;
    } robot_pose_;         ///< 机器人位姿

    void drawRobot(QPainter& painter);
};

#endif // MAP_VIEW_WIDGET_H 