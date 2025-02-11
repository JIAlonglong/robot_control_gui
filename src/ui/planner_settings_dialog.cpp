#include "ui/planner_settings_dialog.h"
#include "ros/robot_controller.h"
#include <QGroupBox>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QDialogButtonBox>

struct PlannerSettingsDialog::Private {
    std::shared_ptr<RobotController> robot_controller;
    QComboBox* global_planner_combo{nullptr};
    QComboBox* local_planner_combo{nullptr};
    QLabel* global_planner_desc{nullptr};
    QLabel* local_planner_desc{nullptr};
    QPushButton* ok_button{nullptr};
    QPushButton* cancel_button{nullptr};
    QDialogButtonBox* button_box{nullptr};
};

PlannerSettingsDialog::PlannerSettingsDialog(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QDialog(parent)
    , d_(std::make_unique<Private>())
{
    d_->robot_controller = robot_controller;
    setupUi();
    connectSignalsAndSlots();
    updatePlannerDescriptions();
}

PlannerSettingsDialog::~PlannerSettingsDialog() = default;

void PlannerSettingsDialog::setupUi()
{
    setWindowTitle(tr("规划器设置"));
    setModal(true);

    auto* layout = new QVBoxLayout(this);

    // 全局规划器
    auto* global_group = new QGroupBox(tr("全局规划器"), this);
    auto* global_layout = new QVBoxLayout(global_group);

    d_->global_planner_combo = new QComboBox(this);
    d_->global_planner_combo->addItems({"navfn", "global_planner", "carrot_planner"});
    d_->global_planner_desc = new QLabel(this);
    d_->global_planner_desc->setWordWrap(true);

    global_layout->addWidget(d_->global_planner_combo);
    global_layout->addWidget(d_->global_planner_desc);

    // 局部规划器
    auto* local_group = new QGroupBox(tr("局部规划器"), this);
    auto* local_layout = new QVBoxLayout(local_group);

    d_->local_planner_combo = new QComboBox(this);
    d_->local_planner_combo->addItems({"dwa_local_planner", "teb_local_planner", "eband_local_planner"});
    d_->local_planner_desc = new QLabel(this);
    d_->local_planner_desc->setWordWrap(true);

    local_layout->addWidget(d_->local_planner_combo);
    local_layout->addWidget(d_->local_planner_desc);

    // 按钮
    d_->button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    d_->ok_button = d_->button_box->button(QDialogButtonBox::Ok);
    d_->cancel_button = d_->button_box->button(QDialogButtonBox::Cancel);

    d_->ok_button->setText(tr("确定"));
    d_->cancel_button->setText(tr("取消"));

    layout->addWidget(global_group);
    layout->addWidget(local_group);
    layout->addWidget(d_->button_box);
}

void PlannerSettingsDialog::connectSignalsAndSlots()
{
    connect(d_->global_planner_combo, &QComboBox::currentTextChanged,
            this, &PlannerSettingsDialog::onGlobalPlannerChanged);
    connect(d_->local_planner_combo, &QComboBox::currentTextChanged,
            this, &PlannerSettingsDialog::onLocalPlannerChanged);

    connect(d_->button_box, &QDialogButtonBox::accepted,
            this, &PlannerSettingsDialog::onAccepted);
    connect(d_->button_box, &QDialogButtonBox::rejected,
            this, &PlannerSettingsDialog::onRejected);
}

void PlannerSettingsDialog::onGlobalPlannerChanged(const QString& planner)
{
    if (d_->robot_controller) {
        d_->robot_controller->setGlobalPlanner(planner);
        updatePlannerDescriptions();
    }
}

void PlannerSettingsDialog::onLocalPlannerChanged(const QString& planner)
{
    if (d_->robot_controller) {
        d_->robot_controller->setLocalPlanner(planner);
        updatePlannerDescriptions();
    }
}

void PlannerSettingsDialog::onAccepted()
{
    if (d_->robot_controller) {
        d_->robot_controller->saveNavigationSettings();
        accept();
    }
}

void PlannerSettingsDialog::onRejected()
{
    if (d_->robot_controller) {
        d_->robot_controller->restoreNavigationSettings();
        reject();
    }
}

void PlannerSettingsDialog::updatePlannerDescriptions()
{
    QString global_planner = d_->global_planner_combo->currentText();
    QString local_planner = d_->local_planner_combo->currentText();

    // 更新全局规划器描述
    QString global_desc;
    if (global_planner == "navfn") {
        global_desc = tr("Dijkstra算法实现的全局路径规划器，适用于大多数场景。");
    } else if (global_planner == "global_planner") {
        global_desc = tr("A*算法实现的全局路径规划器，支持更多配置选项。");
    } else if (global_planner == "carrot_planner") {
        global_desc = tr("简单的直线规划器，适用于开阔环境。");
    }
    d_->global_planner_desc->setText(global_desc);

    // 更新局部规划器描述
    QString local_desc;
    if (local_planner == "dwa_local_planner") {
        local_desc = tr("动态窗口法实现的局部规划器，适用于差分驱动机器人。");
    } else if (local_planner == "teb_local_planner") {
        local_desc = tr("时间弹性带算法实现的局部规划器，支持多种运动约束。");
    } else if (local_planner == "eband_local_planner") {
        local_desc = tr("弹性带算法实现的局部规划器，生成平滑的路径。");
    }
    d_->local_planner_desc->setText(local_desc);
} 