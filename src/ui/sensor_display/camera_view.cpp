#include "ui/sensor_display/camera_view.h"
#include <QMessageBox>
#include <opencv2/imgproc/imgproc.hpp>

CameraView::CameraView(QWidget* parent)
    : QWidget(parent)
    , it_(nh_)
    , is_running_(false)
{
    setupUi();
    setupRosConnections();
    updateTopicList();
}

CameraView::~CameraView()
{
    stop();
}

void CameraView::setupUi()
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
    // 创建顶部控制栏
    QHBoxLayout* control_layout = new QHBoxLayout;
    
    // 话题选择下拉框
    topic_combo_ = new QComboBox(this);
    topic_combo_->setMinimumWidth(200);
    control_layout->addWidget(new QLabel(tr("摄像头话题:")));
    control_layout->addWidget(topic_combo_);
    
    // 刷新按钮
    QPushButton* refresh_btn = new QPushButton(tr("刷新"), this);
    control_layout->addWidget(refresh_btn);
    
    // 启动/停止按钮
    start_stop_btn_ = new QPushButton(tr("启动"), this);
    control_layout->addWidget(start_stop_btn_);
    
    control_layout->addStretch();
    
    // 状态标签
    status_label_ = new QLabel(tr("就绪"), this);
    control_layout->addWidget(status_label_);
    
    layout->addLayout(control_layout);
    
    // 图像显示标签
    image_label_ = new QLabel(this);
    image_label_->setMinimumSize(640, 480);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setStyleSheet("background-color: black;");
    layout->addWidget(image_label_);
    
    // 连接信号
    connect(topic_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &CameraView::onTopicSelected);
    connect(refresh_btn, &QPushButton::clicked,
            this, &CameraView::updateTopicList);
    connect(start_stop_btn_, &QPushButton::clicked,
            this, &CameraView::onStartStopClicked);
}

void CameraView::setupRosConnections()
{
    // ROS连接在选择话题时建立
}

void CameraView::updateTopicList()
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    
    topic_combo_->clear();
    for (const auto& topic : topic_info) {
        if (topic.datatype == "sensor_msgs/Image") {
            topic_combo_->addItem(QString::fromStdString(topic.name));
        }
    }
}

void CameraView::onTopicSelected(const QString& topic)
{
    if (is_running_) {
        stop();
        current_topic_ = topic;
        start();
    } else {
        current_topic_ = topic;
    }
}

void CameraView::onStartStopClicked()
{
    if (is_running_) {
        stop();
    } else {
        start();
    }
}

void CameraView::start()
{
    if (current_topic_.isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先选择摄像头话题"));
        return;
    }
    
    try {
        image_sub_ = it_.subscribe(current_topic_.toStdString(), 1,
                                 &CameraView::imageCallback, this);
        is_running_ = true;
        start_stop_btn_->setText(tr("停止"));
        status_label_->setText(tr("正在接收图像"));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"),
                            tr("启动失败: %1").arg(e.what()));
    }
}

void CameraView::stop()
{
    image_sub_.shutdown();
    is_running_ = false;
    start_stop_btn_->setText(tr("启动"));
    status_label_->setText(tr("已停止"));
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
        
        // 转换为RGB格式（如果需要）
        cv::Mat rgb_image;
        if (cv_ptr->encoding == "bgr8") {
            cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
        } else {
            rgb_image = cv_ptr->image;
        }
        
        // 转换为QImage并显示
        current_image_ = QImage(rgb_image.data, rgb_image.cols, rgb_image.rows,
                              rgb_image.step, QImage::Format_RGB888);
        
        // 缩放图像以适应标签大小
        QSize label_size = image_label_->size();
        image_label_->setPixmap(QPixmap::fromImage(current_image_)
                              .scaled(label_size, Qt::KeepAspectRatio));
                              
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
} 