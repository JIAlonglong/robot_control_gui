#include "mapping_panel.h"
#include "ros/mapping_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QProgressBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>

struct MappingPanel::Private {
    std::shared_ptr<MappingController> mapping_controller;

    // 基本控制按钮
    QPushButton* start_button{nullptr};
    QPushButton* stop_button{nullptr};
    QPushButton* save_button{nullptr};
    QPushButton* pause_button{nullptr};
    QPushButton* resume_button{nullptr};

    // 基本设置控件
    QComboBox* method_combo{nullptr};
    QDoubleSpinBox* resolution_spin{nullptr};
    QSpinBox* update_rate_spin{nullptr};
    QCheckBox* real_time_update_check{nullptr};

    // 自动建图控制按钮
    QPushButton* start_auto_button{nullptr};
    QPushButton* stop_auto_button{nullptr};

    // 自动建图设置控件
    QComboBox* strategy_combo{nullptr};
    QDoubleSpinBox* radius_spin{nullptr};
    QDoubleSpinBox* frontier_size_spin{nullptr};
    QSpinBox* max_time_spin{nullptr};
    QCheckBox* return_home_check{nullptr};

    // 状态显示控件
    QLabel* mapping_status_label{nullptr};
    QProgressBar* mapping_progress_bar{nullptr};
    QProgressBar* exploration_progress_bar{nullptr};

    // 分组框
    QGroupBox* basic_group{nullptr};
    QGroupBox* auto_mapping_group{nullptr};
    QGroupBox* status_group{nullptr};
};

MappingPanel::MappingPanel(QWidget* parent)
    : QWidget(parent)
    , d_(std::make_unique<Private>())
{
    setupUi();
    connectSignals();
}

MappingPanel::~MappingPanel() = default;

void MappingPanel::setMappingController(std::shared_ptr<MappingController> controller)
{
    d_->mapping_controller = controller;
}

void MappingPanel::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    setupBasicGroup();
    setupAutoMappingGroup();
    setupStatusGroup();

    layout->addWidget(d_->basic_group);
    layout->addWidget(d_->auto_mapping_group);
    layout->addWidget(d_->status_group);
}

void MappingPanel::setupBasicGroup()
{
    d_->basic_group = new QGroupBox(tr("基本控制"), this);
    auto* layout = new QVBoxLayout(d_->basic_group);

    // 控制按钮
    auto* button_layout = new QHBoxLayout;
    d_->start_button = new QPushButton(tr("开始建图"), this);
    d_->stop_button = new QPushButton(tr("停止建图"), this);
    d_->save_button = new QPushButton(tr("保存地图"), this);
    d_->pause_button = new QPushButton(tr("暂停"), this);
    d_->resume_button = new QPushButton(tr("继续"), this);

    button_layout->addWidget(d_->start_button);
    button_layout->addWidget(d_->stop_button);
    button_layout->addWidget(d_->save_button);
    button_layout->addWidget(d_->pause_button);
    button_layout->addWidget(d_->resume_button);

    // 设置控件
    auto* settings_layout = new QFormLayout;
    d_->method_combo = new QComboBox(this);
    d_->method_combo->addItems({"gmapping", "cartographer", "hector"});

    d_->resolution_spin = new QDoubleSpinBox(this);
    d_->resolution_spin->setRange(0.01, 1.0);
    d_->resolution_spin->setSingleStep(0.01);
    d_->resolution_spin->setValue(0.05);
    d_->resolution_spin->setSuffix(" m");

    d_->update_rate_spin = new QSpinBox(this);
    d_->update_rate_spin->setRange(1, 60);
    d_->update_rate_spin->setValue(10);
    d_->update_rate_spin->setSuffix(" Hz");

    d_->real_time_update_check = new QCheckBox(tr("实时更新"), this);
    d_->real_time_update_check->setChecked(true);

    settings_layout->addRow(tr("建图方法:"), d_->method_combo);
    settings_layout->addRow(tr("地图分辨率:"), d_->resolution_spin);
    settings_layout->addRow(tr("更新频率:"), d_->update_rate_spin);
    settings_layout->addRow("", d_->real_time_update_check);

    layout->addLayout(button_layout);
    layout->addLayout(settings_layout);
}

void MappingPanel::setupAutoMappingGroup()
{
    d_->auto_mapping_group = new QGroupBox(tr("自动建图"), this);
    auto* layout = new QVBoxLayout(d_->auto_mapping_group);

    // 控制按钮
    auto* button_layout = new QHBoxLayout;
    d_->start_auto_button = new QPushButton(tr("开始自动建图"), this);
    d_->stop_auto_button = new QPushButton(tr("停止自动建图"), this);

    button_layout->addWidget(d_->start_auto_button);
    button_layout->addWidget(d_->stop_auto_button);

    // 设置控件
    auto* settings_layout = new QFormLayout;
    d_->strategy_combo = new QComboBox(this);
    d_->strategy_combo->addItems({"frontier", "random", "spiral"});

    d_->radius_spin = new QDoubleSpinBox(this);
    d_->radius_spin->setRange(0.1, 10.0);
    d_->radius_spin->setSingleStep(0.1);
    d_->radius_spin->setValue(3.0);
    d_->radius_spin->setSuffix(" m");

    d_->frontier_size_spin = new QDoubleSpinBox(this);
    d_->frontier_size_spin->setRange(0.1, 5.0);
    d_->frontier_size_spin->setSingleStep(0.1);
    d_->frontier_size_spin->setValue(0.5);
    d_->frontier_size_spin->setSuffix(" m");

    d_->max_time_spin = new QSpinBox(this);
    d_->max_time_spin->setRange(1, 120);
    d_->max_time_spin->setValue(30);
    d_->max_time_spin->setSuffix(tr(" 分钟"));

    d_->return_home_check = new QCheckBox(tr("完成后返回起点"), this);
    d_->return_home_check->setChecked(true);

    settings_layout->addRow(tr("探索策略:"), d_->strategy_combo);
    settings_layout->addRow(tr("探索半径:"), d_->radius_spin);
    settings_layout->addRow(tr("边界大小:"), d_->frontier_size_spin);
    settings_layout->addRow(tr("最大时间:"), d_->max_time_spin);
    settings_layout->addRow("", d_->return_home_check);

    layout->addLayout(button_layout);
    layout->addLayout(settings_layout);
}

void MappingPanel::setupStatusGroup()
{
    d_->status_group = new QGroupBox(tr("状态"), this);
    auto* layout = new QVBoxLayout(d_->status_group);

    d_->mapping_status_label = new QLabel(tr("建图状态: 未开始"), this);
    d_->mapping_progress_bar = new QProgressBar(this);
    d_->exploration_progress_bar = new QProgressBar(this);

    d_->mapping_progress_bar->setRange(0, 100);
    d_->exploration_progress_bar->setRange(0, 100);

    layout->addWidget(d_->mapping_status_label);
    layout->addWidget(new QLabel(tr("建图进度:"), this));
    layout->addWidget(d_->mapping_progress_bar);
    layout->addWidget(new QLabel(tr("探索进度:"), this));
    layout->addWidget(d_->exploration_progress_bar);
}

void MappingPanel::connectSignals()
{
    // 基本控制按钮
    connect(d_->start_button, &QPushButton::clicked, this, &MappingPanel::onStartMapping);
    connect(d_->stop_button, &QPushButton::clicked, this, &MappingPanel::onStopMapping);
    connect(d_->save_button, &QPushButton::clicked, this, &MappingPanel::onSaveMap);
    connect(d_->pause_button, &QPushButton::clicked, this, &MappingPanel::onPauseMapping);
    connect(d_->resume_button, &QPushButton::clicked, this, &MappingPanel::onResumeMapping);

    // 设置控件
    connect(d_->method_combo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &MappingPanel::onMethodChanged);
    connect(d_->resolution_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MappingPanel::onResolutionChanged);
    connect(d_->update_rate_spin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MappingPanel::onUpdateRateChanged);

    // 自动建图设置
    connect(d_->strategy_combo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &MappingPanel::onStrategyChanged);
    connect(d_->radius_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MappingPanel::onRadiusChanged);
    connect(d_->frontier_size_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MappingPanel::onFrontierSizeChanged);
    connect(d_->max_time_spin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MappingPanel::onMaxTimeChanged);

    // 复选框
    connect(d_->return_home_check, &QCheckBox::stateChanged, this, &MappingPanel::onReturnHomeChanged);
    connect(d_->real_time_update_check, &QCheckBox::stateChanged, this, &MappingPanel::onRealTimeUpdateChanged);

    // 自动建图控制按钮
    connect(d_->start_auto_button, &QPushButton::clicked, this, &MappingPanel::onStartAutoMapping);
    connect(d_->stop_auto_button, &QPushButton::clicked, this, &MappingPanel::onStopAutoMapping);
}

void MappingPanel::updateButtonStates(bool is_mapping)
{
    d_->start_button->setEnabled(!is_mapping);
    d_->stop_button->setEnabled(is_mapping);
    d_->save_button->setEnabled(!is_mapping);
    d_->pause_button->setEnabled(is_mapping);
    d_->resume_button->setEnabled(false);

    d_->method_combo->setEnabled(!is_mapping);
    d_->resolution_spin->setEnabled(!is_mapping);
    d_->update_rate_spin->setEnabled(!is_mapping);
    d_->real_time_update_check->setEnabled(!is_mapping);

    d_->start_auto_button->setEnabled(!is_mapping);
    d_->auto_mapping_group->setEnabled(!is_mapping);
}

void MappingPanel::updateAutoMappingButtonStates(bool is_auto_mapping)
{
    d_->start_auto_button->setEnabled(!is_auto_mapping);
    d_->stop_auto_button->setEnabled(is_auto_mapping);
    d_->start_button->setEnabled(!is_auto_mapping);
    d_->stop_button->setEnabled(is_auto_mapping);
    d_->save_button->setEnabled(!is_auto_mapping);
    d_->pause_button->setEnabled(is_auto_mapping);
    d_->resume_button->setEnabled(false);

    d_->method_combo->setEnabled(!is_auto_mapping);
    d_->resolution_spin->setEnabled(!is_auto_mapping);
    d_->update_rate_spin->setEnabled(!is_auto_mapping);
    d_->real_time_update_check->setEnabled(!is_auto_mapping);
    d_->strategy_combo->setEnabled(!is_auto_mapping);
    d_->radius_spin->setEnabled(!is_auto_mapping);
    d_->frontier_size_spin->setEnabled(!is_auto_mapping);
    d_->max_time_spin->setEnabled(!is_auto_mapping);
    d_->return_home_check->setEnabled(!is_auto_mapping);
}

void MappingPanel::onStartMapping()
{
    if (!d_->mapping_controller) {
        QMessageBox::warning(this, tr("错误"), tr("建图控制器未初始化"));
        return;
    }

    QString method = d_->method_combo->currentText();
    QString map_name = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    double resolution = d_->resolution_spin->value();
    bool real_time_update = d_->real_time_update_check->isChecked();

    if (d_->mapping_controller->startMapping(method, map_name, "", resolution, real_time_update, QVariantMap())) {
        updateButtonStates(true);
        d_->mapping_status_label->setText(tr("建图状态: 进行中"));
    } else {
        QMessageBox::warning(this, tr("错误"), tr("启动建图失败"));
    }
}

void MappingPanel::onStopMapping()
{
    if (!d_->mapping_controller) {
        return;
    }

    d_->mapping_controller->stopMapping();
    updateButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 已停止"));
}

void MappingPanel::onSaveMap()
{
    if (!d_->mapping_controller) {
        QMessageBox::warning(this, tr("错误"), tr("建图控制器未初始化"));
        return;
    }

    QString file_path = QFileDialog::getSaveFileName(this,
        tr("保存地图"), "",
        tr("地图文件 (*.pgm *.yaml);;所有文件 (*.*)"));

    if (file_path.isEmpty()) {
        return;
    }

    d_->mapping_controller->saveMap(file_path);
    QMessageBox::information(this, tr("成功"), tr("地图已保存"));
}

void MappingPanel::onStartAutoMapping()
{
    if (!d_->mapping_controller) {
        QMessageBox::warning(this, tr("错误"), tr("建图控制器未初始化"));
        return;
    }

    QString method = d_->method_combo->currentText();
    double resolution = d_->resolution_spin->value();

    if (d_->mapping_controller->startAutoMapping(method, resolution)) {
        updateAutoMappingButtonStates(true);
        d_->mapping_status_label->setText(tr("建图状态: 自动建图中"));
    } else {
        QMessageBox::warning(this, tr("错误"), tr("启动自动建图失败"));
    }
}

void MappingPanel::onStopAutoMapping()
{
    if (!d_->mapping_controller) {
        return;
    }

    d_->mapping_controller->stopAutoMapping();
    updateAutoMappingButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 已停止"));
}

void MappingPanel::onPauseMapping()
{
    if (d_->mapping_controller) {
        d_->mapping_controller->pauseMapping();
        d_->mapping_status_label->setText(tr("建图状态: 已暂停"));
        d_->pause_button->setEnabled(false);
        d_->resume_button->setEnabled(true);
    }
}

void MappingPanel::onResumeMapping()
{
    if (d_->mapping_controller) {
        d_->mapping_controller->resumeMapping();
        d_->mapping_status_label->setText(tr("建图状态: 进行中"));
        d_->pause_button->setEnabled(true);
        d_->resume_button->setEnabled(false);
    }
}

void MappingPanel::onMethodChanged(const QString& method)
{
    // 更新方法相关的UI状态
    bool is_auto = method.contains("auto", Qt::CaseInsensitive);
    d_->auto_mapping_group->setEnabled(is_auto);
}

void MappingPanel::onResolutionChanged(double value)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setMapResolution(value);
    }
}

void MappingPanel::onUpdateRateChanged(int value)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setUpdateRate(value);
    }
}

void MappingPanel::onStrategyChanged(const QString& strategy)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setExplorationStrategy(strategy);
    }
}

void MappingPanel::onRadiusChanged(double value)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setExplorationRadius(value);
    }
}

void MappingPanel::onFrontierSizeChanged(double value)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setFrontierSize(value);
    }
}

void MappingPanel::onMaxTimeChanged(int value)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setMaxExplorationTime(value * 60); // 转换为秒
    }
}

void MappingPanel::onReturnHomeChanged(int state)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setReturnHomeEnabled(state == Qt::Checked);
    }
}

void MappingPanel::onRealTimeUpdateChanged(int state)
{
    if (d_->mapping_controller) {
        d_->mapping_controller->setRealTimeUpdateEnabled(state == Qt::Checked);
    }
}

void MappingPanel::onMappingStarted()
{
    updateButtonStates(true);
    d_->mapping_status_label->setText(tr("建图状态: 进行中"));
    d_->mapping_progress_bar->setValue(0);
    d_->exploration_progress_bar->setValue(0);
}

void MappingPanel::onMappingProgressUpdated(double progress)
{
    d_->mapping_progress_bar->setValue(static_cast<int>(progress * 100));
}

void MappingPanel::onExplorationProgressUpdated(double progress)
{
    d_->exploration_progress_bar->setValue(static_cast<int>(progress * 100));
}

void MappingPanel::onMappingStateChanged(const QString& state)
{
    d_->mapping_status_label->setText(tr("建图状态: %1").arg(state));
}

void MappingPanel::onMappingProgressChanged(double progress)
{
    d_->mapping_progress_bar->setValue(static_cast<int>(progress * 100));
}

void MappingPanel::onMappingFailed(const QString& error)
{
    QMessageBox::warning(this, tr("错误"), error);
    updateButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 失败"));
}

void MappingPanel::onMappingFinished()
{
    updateButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 完成"));
}

void MappingPanel::onMapSaved(const QString& filename)
{
    QMessageBox::information(this, tr("成功"), tr("地图已保存到: %1").arg(filename));
}

void MappingPanel::onExplorationProgressChanged(double progress)
{
    d_->exploration_progress_bar->setValue(static_cast<int>(progress * 100));
}

void MappingPanel::onReturnHomeStarted()
{
    d_->mapping_status_label->setText(tr("建图状态: 正在返回起点"));
}

void MappingPanel::onReturnHomeCompleted()
{
    d_->mapping_status_label->setText(tr("建图状态: 已返回起点"));
}

void MappingPanel::onExplorationCompleted()
{
    d_->mapping_status_label->setText(tr("建图状态: 探索完成"));
}

void MappingPanel::onExplorationFailed(const QString& error)
{
    QMessageBox::warning(this, tr("错误"), error);
    updateButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 探索失败"));
}

void MappingPanel::onMappingCancelled()
{
    updateButtonStates(false);
    d_->mapping_status_label->setText(tr("建图状态: 已取消"));
}

void MappingPanel::onMappingStatusUpdated(const QString& status)
{
    d_->mapping_status_label->setText(tr("建图状态: %1").arg(status));
} 