#include "ui/sensor_display/camera_view.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <ros/master.h>

CameraView::CameraView(QWidget* parent)
    : QWidget(parent)
    , it_(nh_)
    , is_running_(false)
{
    setupUi();
    setupRosConnections();
}

CameraView::~CameraView()
{
    if (is_running_) {
        stop();
    }
}

void CameraView::setupUi()
{
    // 创建主布局
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);

    // 创建控制面板
    auto* control_widget = new QWidget(this);
    control_widget->setStyleSheet(
        "QWidget {"
        "    background: #f8f9fa;"
        "    border-bottom: 1px solid #dee2e6;"
        "}"
        "QLabel {"
        "    padding: 5px;"
        "}"
        "QComboBox {"
        "    padding: 5px;"
        "    border: 1px solid #ced4da;"
        "    border-radius: 4px;"
        "    min-width: 200px;"
        "}"
        "QPushButton {"
        "    padding: 5px 15px;"
        "    border: none;"
        "    border-radius: 4px;"
        "    background: #007bff;"
        "    color: white;"
        "}"
        "QPushButton:hover {"
        "    background: #0056b3;"
        "}"
        "QPushButton:pressed {"
        "    background: #004085;"
        "}"
    );

    auto* control_layout = new QHBoxLayout(control_widget);
    control_layout->setContentsMargins(10, 5, 10, 5);
    control_layout->setSpacing(10);

    // 添加话题选择组件
    control_layout->addWidget(new QLabel(tr("摄像头话题:")));
    topic_combo_ = new QComboBox(this);
    control_layout->addWidget(topic_combo_);

    // 添加启动/停止按钮
    start_stop_btn_ = new QPushButton(tr("启动"), this);
    control_layout->addWidget(start_stop_btn_);

    // 添加状态标签
    status_label_ = new QLabel(tr("未连接"), this);
    control_layout->addWidget(status_label_);

    control_layout->addStretch();

    // 创建图像显示标签
    image_label_ = new QLabel(this);
    image_label_->setMinimumSize(640, 480);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setStyleSheet(
        "QLabel {"
        "    background: #343a40;"
        "    color: white;"
        "    border: none;"
        "}"
    );
    image_label_->setText(tr("等待图像..."));

    // 添加到主布局
    main_layout->addWidget(control_widget);
    main_layout->addWidget(image_label_, 1);

    // 连接信号
    connect(topic_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &CameraView::onTopicSelected);
    connect(start_stop_btn_, &QPushButton::clicked,
            this, &CameraView::onStartStopClicked);

    // 更新话题列表
    updateTopicList();
}

void CameraView::setupRosConnections()
{
    // ROS连接在选择话题时建立
}

void CameraView::updateTopicList()
{
    topic_combo_->clear();
    
    // 获取所有话题
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    
    QStringList image_topics;
    
    // 过滤图像话题
    for (const auto& topic : topic_info) {
        if (topic.datatype == "sensor_msgs/Image") {
            image_topics << QString::fromStdString(topic.name);
        }
    }
    
    // 如果没有找到图像话题
    if (image_topics.isEmpty()) {
        status_label_->setText(tr("未找到图像话题"));
        start_stop_btn_->setEnabled(false);
        return;
    }
    
    // 添加话题到下拉框
    topic_combo_->addItems(image_topics);
    start_stop_btn_->setEnabled(true);
}

void CameraView::onTopicSelected(const QString& topic)
{
    if (is_running_) {
        stop();
    }
    current_topic_ = topic;
}

void CameraView::onStartStopClicked()
{
    if (!is_running_) {
        start();
    } else {
        stop();
    }
}

void CameraView::start()
{
    if (current_topic_.isEmpty()) {
        QMessageBox::warning(this, tr("错误"), tr("请先选择一个图像话题"));
        return;
    }

    try {
        image_sub_ = it_.subscribe(current_topic_.toStdString(), 1,
                                 &CameraView::imageCallback, this);
        is_running_ = true;
        start_stop_btn_->setText(tr("停止"));
        status_label_->setText(tr("已连接"));
        topic_combo_->setEnabled(false);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"),
                            tr("无法订阅话题: %1").arg(e.what()));
    }
}

void CameraView::stop()
{
    image_sub_.shutdown();
    is_running_ = false;
    start_stop_btn_->setText(tr("启动"));
    status_label_->setText(tr("未连接"));
    topic_combo_->setEnabled(true);
    image_label_->setText(tr("等待图像..."));
}

void CameraView::setTopic(const QString& topic)
{
    int index = topic_combo_->findText(topic);
    if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
    }
}

void CameraView::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        
        // 将OpenCV图像转换为QImage
        QImage image;
        if (cv_ptr->encoding == "bgr8") {
            image = QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows,
                         cv_ptr->image.step, QImage::Format_RGB888).rgbSwapped();
        } else if (cv_ptr->encoding == "mono8") {
            image = QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows,
                         cv_ptr->image.step, QImage::Format_Grayscale8);
        } else {
            ROS_ERROR("Unsupported image encoding: %s", cv_ptr->encoding.c_str());
            return;
        }
        
        // 缩放图像以适应标签大小
        current_image_ = image.scaled(image_label_->size(),
                                    Qt::KeepAspectRatio,
                                    Qt::SmoothTransformation);
        
        // 更新图像显示
        image_label_->setPixmap(QPixmap::fromImage(current_image_));
        
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
} 