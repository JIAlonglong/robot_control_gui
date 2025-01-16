#include "ui/sensor_display/camera_view.h"
<<<<<<< HEAD
#include <QTimer>
#include <QMessageBox>
=======
#include <QMessageBox>
#include <opencv2/imgproc/imgproc.hpp>
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14

CameraView::CameraView(QWidget* parent)
    : QWidget(parent)
    , it_(nh_)
    , is_running_(false)
{
    setupUi();
    setupRosConnections();
<<<<<<< HEAD

    // 创建定时器用于更新话题列表
    QTimer* update_timer = new QTimer(this);
    connect(update_timer, &QTimer::timeout, this, &CameraView::updateTopicList);
    update_timer->start(5000);  // 每5秒更新一次
=======
    updateTopicList();
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
}

CameraView::~CameraView()
{
    stop();
}

void CameraView::setupUi()
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
<<<<<<< HEAD
    // 创建话题选择组件
    topic_combo_ = new QComboBox(this);
    topic_combo_->setEditable(true);
    topic_combo_->setInsertPolicy(QComboBox::InsertAlphabetically);
    layout->addWidget(topic_combo_);
    
    // 创建开始/停止按钮
    start_stop_btn_ = new QPushButton(tr("开始"), this);
    layout->addWidget(start_stop_btn_);
    
    // 创建图像显示标签
    image_label_ = new QLabel(this);
    image_label_->setMinimumSize(320, 240);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setText(tr("等待图像..."));
    layout->addWidget(image_label_);
    
    // 创建状态标签
    status_label_ = new QLabel(tr("就绪"), this);
    layout->addWidget(status_label_);
    
    // 连接信号
    connect(topic_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &CameraView::onTopicSelected);
=======
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
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    connect(start_stop_btn_, &QPushButton::clicked,
            this, &CameraView::onStartStopClicked);
}

void CameraView::setupRosConnections()
{
<<<<<<< HEAD
    updateTopicList();
=======
    // ROS连接在选择话题时建立
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
}

void CameraView::updateTopicList()
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    
<<<<<<< HEAD
    QStringList image_topics;
    for (const auto& topic : topic_info) {
        if (topic.datatype == "sensor_msgs/Image") {
            image_topics << QString::fromStdString(topic.name);
        }
    }
    
    // 保存当前选择的话题
    QString current_text = topic_combo_->currentText();
    
    // 更新话题列表
    topic_combo_->clear();
    topic_combo_->addItems(image_topics);
    
    // 恢复之前选择的话题
    int index = topic_combo_->findText(current_text);
    if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
=======
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
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    }
}

void CameraView::start()
{
<<<<<<< HEAD
    if (is_running_) return;
    
    QString topic = topic_combo_->currentText();
    if (topic.isEmpty()) {
        QMessageBox::warning(this, tr("错误"), tr("请选择一个图像话题"));
=======
    if (current_topic_.isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先选择摄像头话题"));
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
        return;
    }
    
    try {
<<<<<<< HEAD
        image_sub_ = it_.subscribe(topic.toStdString(), 1,
            &CameraView::imageCallback, this);
        is_running_ = true;
        start_stop_btn_->setText(tr("停止"));
        status_label_->setText(tr("正在接收图像..."));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"),
            tr("订阅话题失败: %1").arg(e.what()));
=======
        image_sub_ = it_.subscribe(current_topic_.toStdString(), 1,
                                 &CameraView::imageCallback, this);
        is_running_ = true;
        start_stop_btn_->setText(tr("停止"));
        status_label_->setText(tr("正在接收图像"));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"),
                            tr("启动失败: %1").arg(e.what()));
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    }
}

void CameraView::stop()
{
<<<<<<< HEAD
    if (!is_running_) return;
    
    image_sub_.shutdown();
    is_running_ = false;
    start_stop_btn_->setText(tr("开始"));
=======
    image_sub_.shutdown();
    is_running_ = false;
    start_stop_btn_->setText(tr("启动"));
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    status_label_->setText(tr("已停止"));
}

void CameraView::setTopic(const QString& topic)
{
    int index = topic_combo_->findText(topic);
    if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
<<<<<<< HEAD
    } else {
        topic_combo_->setEditText(topic);
    }
}

void CameraView::onTopicSelected(const QString& topic)
{
    if (is_running_) {
        stop();
        start();
    }
}

void CameraView::onStartStopClicked()
{
    if (is_running_) {
        stop();
    } else {
        start();
=======
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    }
}

void CameraView::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
<<<<<<< HEAD
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        QImage::Format format;
        
        switch (cv_ptr->encoding[0]) {
            case 'm':  // mono8, mono16
                format = QImage::Format_Grayscale8;
                break;
            case 'b':  // bgr8, bgra8
            case 'r':  // rgb8, rgba8
                format = QImage::Format_RGB888;
                break;
            default:
                status_label_->setText(tr("不支持的图像格式: %1")
                    .arg(QString::fromStdString(cv_ptr->encoding)));
                return;
        }
        
        // 转换为QImage
        QImage image(cv_ptr->image.data,
                    cv_ptr->image.cols,
                    cv_ptr->image.rows,
                    cv_ptr->image.step[0],
                    format);
        
        // 如果是BGR格式，需要转换为RGB
        if (cv_ptr->encoding == "bgr8") {
            image = image.rgbSwapped();
        }
        
        // 缩放图像以适应标签大小
        current_image_ = image.scaled(image_label_->size(),
                                    Qt::KeepAspectRatio,
                                    Qt::SmoothTransformation);
        
        // 更新显示
        image_label_->setPixmap(QPixmap::fromImage(current_image_));
        status_label_->setText(tr("图像大小: %1x%2")
            .arg(image.width())
            .arg(image.height()));
            
    } catch (const cv_bridge::Exception& e) {
        status_label_->setText(tr("图像转换错误: %1").arg(e.what()));
=======
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
>>>>>>> c5b17e6c1d8ee1290c44112ad118346e37047d14
    }
} 