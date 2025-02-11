#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>

class CameraView : public QWidget
{
    Q_OBJECT

public:
    explicit CameraView(QWidget* parent = nullptr);
    ~CameraView();

    void setTopic(const QString& topic);
    void start();
    void stop();

private Q_SLOTS:
    void onTopicSelected(const QString& topic);
    void onStartStopClicked();
    void updateTopicList();

private:
    void setupUi();
    void setupRosConnections();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle                 nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber     image_sub_;

    QComboBox*   topic_combo_;
    QPushButton* start_stop_btn_;
    QLabel*      image_label_;
    QLabel*      status_label_;

    QImage  current_image_;
    bool    is_running_;
    QString current_topic_;
};

#endif  // CAMERA_VIEW_H