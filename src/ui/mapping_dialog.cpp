#include "ui/mapping_dialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>

MappingDialog::MappingDialog(QWidget* parent)
    : QDialog(parent)
{
    setupUi();
    setupConnections();
    updateParameterVisibility();
}

void MappingDialog::setupUi()
{
    setWindowTitle("建图设置");
    setMinimumWidth(400);

    auto* main_layout = new QVBoxLayout(this);

    // 建图方法选择
    auto* method_layout = new QHBoxLayout;
    method_layout->addWidget(new QLabel("建图方法:"));
    method_combo_ = new QComboBox;
    method_combo_->addItem("Gmapping (激光SLAM)", static_cast<int>(MappingMethod::GMAPPING));
    method_combo_->addItem("Cartographer (Google SLAM)", static_cast<int>(MappingMethod::CARTOGRAPHER));
    method_combo_->addItem("Hector SLAM (无里程计)", static_cast<int>(MappingMethod::HECTOR_SLAM));
    method_combo_->addItem("RTAB-Map (视觉SLAM)", static_cast<int>(MappingMethod::RTAB_MAP));
    method_layout->addWidget(method_combo_);
    main_layout->addLayout(method_layout);

    // 地图名称和保存路径
    auto* name_layout = new QHBoxLayout;
    name_layout->addWidget(new QLabel("地图名称:"));
    map_name_edit_ = new QLineEdit;
    map_name_edit_->setPlaceholderText("输入地图名称");
    name_layout->addWidget(map_name_edit_);
    main_layout->addLayout(name_layout);

    auto* path_layout = new QHBoxLayout;
    path_layout->addWidget(new QLabel("保存路径:"));
    map_path_edit_ = new QLineEdit;
    map_path_edit_->setText(QDir::currentPath() + "/maps");
    path_layout->addWidget(map_path_edit_);
    browse_button_ = new QPushButton("浏览...");
    path_layout->addWidget(browse_button_);
    main_layout->addLayout(path_layout);

    // 通用参数
    auto* common_layout = new QGridLayout;
    common_layout->addWidget(new QLabel("分辨率(米/像素):"), 0, 0);
    resolution_spin_ = new QDoubleSpinBox;
    resolution_spin_->setRange(0.01, 1.0);
    resolution_spin_->setValue(0.05);
    resolution_spin_->setSingleStep(0.01);
    common_layout->addWidget(resolution_spin_, 0, 1);

    common_layout->addWidget(new QLabel("最大激光范围(米):"), 1, 0);
    max_range_spin_ = new QDoubleSpinBox;
    max_range_spin_->setRange(1.0, 30.0);
    max_range_spin_->setValue(10.0);
    max_range_spin_->setSingleStep(0.5);
    common_layout->addWidget(max_range_spin_, 1, 1);

    real_time_check_ = new QCheckBox("实时更新地图显示");
    real_time_check_->setChecked(true);
    common_layout->addWidget(real_time_check_, 2, 0, 1, 2);
    main_layout->addLayout(common_layout);

    // Gmapping 特定参数
    gmapping_widget_ = new QWidget;
    auto* gmapping_layout = new QGridLayout(gmapping_widget_);
    gmapping_layout->addWidget(new QLabel("粒子数量:"), 0, 0);
    particle_count_spin_ = new QSpinBox;
    particle_count_spin_->setRange(10, 100);
    particle_count_spin_->setValue(30);
    gmapping_layout->addWidget(particle_count_spin_, 0, 1);

    gmapping_layout->addWidget(new QLabel("最小匹配分数:"), 1, 0);
    minimum_score_spin_ = new QDoubleSpinBox;
    minimum_score_spin_->setRange(0.0, 1.0);
    minimum_score_spin_->setValue(0.5);
    minimum_score_spin_->setSingleStep(0.1);
    gmapping_layout->addWidget(minimum_score_spin_, 1, 1);
    main_layout->addWidget(gmapping_widget_);

    // Hector SLAM 特定参数
    hector_widget_ = new QWidget;
    auto* hector_layout = new QVBoxLayout(hector_widget_);
    use_odom_check_ = new QCheckBox("使用里程计数据(可选)");
    use_odom_check_->setChecked(true);
    hector_layout->addWidget(use_odom_check_);
    main_layout->addWidget(hector_widget_);

    // 确定和取消按钮
    auto* button_layout = new QHBoxLayout;
    ok_button_ = new QPushButton("确定");
    cancel_button_ = new QPushButton("取消");
    button_layout->addWidget(ok_button_);
    button_layout->addWidget(cancel_button_);
    main_layout->addLayout(button_layout);
}

void MappingDialog::setupConnections()
{
    connect(method_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MappingDialog::onMethodChanged);
    connect(browse_button_, &QPushButton::clicked,
            this, &MappingDialog::onBrowsePath);
    connect(ok_button_, &QPushButton::clicked,
            this, &QDialog::accept);
    connect(cancel_button_, &QPushButton::clicked,
            this, &QDialog::reject);
    connect(map_name_edit_, &QLineEdit::textChanged,
            this, &MappingDialog::validateInput);
}

void MappingDialog::onMethodChanged(int index)
{
    updateParameterVisibility();
}

void MappingDialog::onBrowsePath()
{
    QString dir = QFileDialog::getExistingDirectory(this, "选择地图保存目录",
                                                  map_path_edit_->text(),
                                                  QFileDialog::ShowDirsOnly |
                                                  QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) {
        map_path_edit_->setText(dir);
    }
}

void MappingDialog::validateInput()
{
    bool valid = !map_name_edit_->text().isEmpty();
    ok_button_->setEnabled(valid);
}

void MappingDialog::updateParameterVisibility()
{
    MappingMethod method = static_cast<MappingMethod>(method_combo_->currentData().toInt());
    
    gmapping_widget_->setVisible(method == MappingMethod::GMAPPING);
    hector_widget_->setVisible(method == MappingMethod::HECTOR_SLAM);
}

MappingDialog::MappingMethod MappingDialog::getSelectedMethod() const
{
    return static_cast<MappingMethod>(method_combo_->currentData().toInt());
}

QString MappingDialog::getMapName() const
{
    return map_name_edit_->text();
}

QString MappingDialog::getMapPath() const
{
    return map_path_edit_->text();
}

double MappingDialog::getResolution() const
{
    return resolution_spin_->value();
}

bool MappingDialog::getUpdateInRealTime() const
{
    return real_time_check_->isChecked();
}

int MappingDialog::getParticleCount() const
{
    return particle_count_spin_->value();
}

double MappingDialog::getMinimumScore() const
{
    return minimum_score_spin_->value();
}

double MappingDialog::getMaximumRange() const
{
    return max_range_spin_->value();
}

bool MappingDialog::getUseOdom() const
{
    return use_odom_check_->isChecked();
} 