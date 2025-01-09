#include "ui/sensor_display/camera_view.h"
#include <QTimer>
#include <QMessageBox>

CameraView::CameraView(QWidget* parent)
    : QWidget(parent)
    , it_(nh_)
    , is_running_(false)
{
    setupUi();
    setupRosConnections();

    // 创建定时器用于更新话题列表
    QTimer* update_timer = new QTimer(this);
    connect(update_timer, &QTimer::timeout, this, &CameraView::updateTopicList);
    update_timer->start(5000);  // 每5秒更新一次
}

CameraView::~CameraView()
{
    stop();
}

void CameraView::setupUi()
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
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
    connect(start_stop_btn_, &QPushButton::clicked,
            this, &CameraView::onStartStopClicked);
}

void CameraView::setupRosConnections()
{
    updateTopicList();
}

void CameraView::updateTopicList()
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    
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
    }
}

void CameraView::start()
{
    if (is_running_) return;
    
    QString topic = topic_combo_->currentText();
    if (topic.isEmpty()) {
        QMessageBox::warning(this, tr("错误"), tr("请选择一个图像话题"));
        return;
    }
    
    try {
        image_sub_ = it_.subscribe(topic.toStdString(), 1,
            &CameraView::imageCallback, this);
        is_running_ = true;
        start_stop_btn_->setText(tr("停止"));
        status_label_->setText(tr("正在接收图像..."));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"),
            tr("订阅话题失败: %1").arg(e.what()));
    }
}

void CameraView::stop()
{
    if (!is_running_) return;
    
    image_sub_.shutdown();
    is_running_ = false;
    start_stop_btn_->setText(tr("开始"));
    status_label_->setText(tr("已停止"));
}

void CameraView::setTopic(const QString& topic)
{
    int index = topic_combo_->findText(topic);
    if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
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
    }
}

void CameraView::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
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
    }
} 