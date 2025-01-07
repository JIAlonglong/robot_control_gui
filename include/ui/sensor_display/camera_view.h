#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class CameraView : public QWidget {
    Q_OBJECT

public:
    explicit CameraView(QWidget* parent = nullptr);
    ~CameraView();

    void start();
    void stop();
    void setTopic(const QString& topic);

private slots:
    void onTopicSelected(const QString& topic);
    void onStartStopClicked();
    void updateTopicList();

private:
    void setupUi();
    void setupRosConnections();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    // ROS相关
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // UI组件
    QLabel* image_label_;
    QComboBox* topic_combo_;
    QPushButton* start_stop_btn_;
    QLabel* status_label_;
    
    // 状态变量
    bool is_running_;
    QString current_topic_;
    QImage current_image_;
};

#endif // CAMERA_VIEW_H 