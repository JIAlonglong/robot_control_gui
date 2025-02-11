#include "ui/settings_widget.h"
#include "ui/action_block.h"
#include "ui/navigation_action_block.h"
#include "ui/loop_action_block.h"
#include "ui/condition_action_block.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>

struct SettingsWidget::Private
{
    ActionBlock* current_action{nullptr};
    QStackedWidget* stacked_widget{nullptr};
    
    // 导航动作设置
    QWidget* navigation_page{nullptr};
    QDoubleSpinBox* x_spinbox{nullptr};
    QDoubleSpinBox* y_spinbox{nullptr};
    QDoubleSpinBox* angle_spinbox{nullptr};
    QDoubleSpinBox* tolerance_spinbox{nullptr};
    
    // 循环动作设置
    QWidget* loop_page{nullptr};
    QSpinBox* count_spinbox{nullptr};
    QComboBox* action_combobox{nullptr};
    
    // 条件动作设置
    QWidget* condition_page{nullptr};
    QComboBox* condition_type_combobox{nullptr};
    QDoubleSpinBox* threshold_spinbox{nullptr};
    QComboBox* true_action_combobox{nullptr};
    QComboBox* false_action_combobox{nullptr};
};

SettingsWidget::SettingsWidget(QWidget* parent)
    : QWidget(parent)
    , d_(std::make_unique<Private>())
{
    setupUi();
}

SettingsWidget::~SettingsWidget() = default;

void SettingsWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    // 创建标题标签
    auto* title_label = new QLabel(tr("动作设置"), this);
    title_label->setStyleSheet("QLabel { font-size: 14pt; font-weight: bold; }");
    layout->addWidget(title_label);
    
    // 创建堆叠部件
    d_->stacked_widget = new QStackedWidget(this);
    layout->addWidget(d_->stacked_widget);
    
    // 创建各类动作设置页面
    setupNavigationSettings();
    setupLoopSettings();
    setupConditionSettings();
    
    // 添加应用按钮
    auto* button_layout = new QHBoxLayout;
    auto* apply_button = new QPushButton(tr("应用"), this);
    connect(apply_button, &QPushButton::clicked, this, &SettingsWidget::updateSettings);
    button_layout->addStretch();
    button_layout->addWidget(apply_button);
    layout->addLayout(button_layout);
}

void SettingsWidget::setupNavigationSettings()
{
    d_->navigation_page = new QWidget(this);
    auto* layout = new QFormLayout(d_->navigation_page);
    
    d_->x_spinbox = new QDoubleSpinBox(d_->navigation_page);
    d_->x_spinbox->setRange(-1000.0, 1000.0);
    d_->x_spinbox->setSingleStep(0.1);
    layout->addRow(tr("X坐标 (m):"), d_->x_spinbox);
    
    d_->y_spinbox = new QDoubleSpinBox(d_->navigation_page);
    d_->y_spinbox->setRange(-1000.0, 1000.0);
    d_->y_spinbox->setSingleStep(0.1);
    layout->addRow(tr("Y坐标 (m):"), d_->y_spinbox);
    
    d_->angle_spinbox = new QDoubleSpinBox(d_->navigation_page);
    d_->angle_spinbox->setRange(-M_PI, M_PI);
    d_->angle_spinbox->setSingleStep(0.1);
    layout->addRow(tr("角度 (rad):"), d_->angle_spinbox);
    
    d_->tolerance_spinbox = new QDoubleSpinBox(d_->navigation_page);
    d_->tolerance_spinbox->setRange(0.01, 1.0);
    d_->tolerance_spinbox->setSingleStep(0.01);
    d_->tolerance_spinbox->setValue(0.1);
    layout->addRow(tr("容差 (m):"), d_->tolerance_spinbox);
    
    d_->stacked_widget->addWidget(d_->navigation_page);

    connect(d_->x_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
    connect(d_->y_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
    connect(d_->angle_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
    connect(d_->tolerance_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
}

void SettingsWidget::setupLoopSettings()
{
    d_->loop_page = new QWidget(this);
    auto* layout = new QFormLayout(d_->loop_page);
    
    d_->count_spinbox = new QSpinBox(d_->loop_page);
    d_->count_spinbox->setRange(1, 1000);
    layout->addRow(tr("循环次数:"), d_->count_spinbox);
    
    d_->action_combobox = new QComboBox(d_->loop_page);
    layout->addRow(tr("循环动作:"), d_->action_combobox);
    
    d_->stacked_widget->addWidget(d_->loop_page);

    connect(d_->action_combobox, &QComboBox::currentTextChanged,
            this, &SettingsWidget::updateSettings);
    connect(d_->count_spinbox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
}

void SettingsWidget::setupConditionSettings()
{
    d_->condition_page = new QWidget(this);
    auto* layout = new QFormLayout(d_->condition_page);
    
    d_->condition_type_combobox = new QComboBox(d_->condition_page);
    d_->condition_type_combobox->addItems({
        tr("机器人位置"),
        tr("机器人朝向"),
        tr("传感器值"),
        tr("障碍物检测"),
        tr("电池电量")
    });
    layout->addRow(tr("条件类型:"), d_->condition_type_combobox);
    
    d_->threshold_spinbox = new QDoubleSpinBox(d_->condition_page);
    d_->threshold_spinbox->setRange(0.0, 1000.0);
    layout->addRow(tr("阈值:"), d_->threshold_spinbox);
    
    d_->true_action_combobox = new QComboBox(d_->condition_page);
    layout->addRow(tr("条件为真时:"), d_->true_action_combobox);
    
    d_->false_action_combobox = new QComboBox(d_->condition_page);
    layout->addRow(tr("条件为假时:"), d_->false_action_combobox);
    
    d_->stacked_widget->addWidget(d_->condition_page);

    connect(d_->condition_type_combobox, &QComboBox::currentTextChanged,
            this, &SettingsWidget::updateSettings);
    connect(d_->threshold_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsWidget::updateSettings);
    connect(d_->true_action_combobox, &QComboBox::currentTextChanged,
            this, &SettingsWidget::updateSettings);
    connect(d_->false_action_combobox, &QComboBox::currentTextChanged,
            this, &SettingsWidget::updateSettings);
}

void SettingsWidget::setActionBlock(ActionBlock* action)
{
    if (!action) {
        clearSettings();
        return;
    }
    
    d_->current_action = action;
    
    if (auto* nav_action = dynamic_cast<NavigationActionBlock*>(action)) {
        d_->stacked_widget->setCurrentWidget(d_->navigation_page);
        d_->x_spinbox->setValue(nav_action->x());
        d_->y_spinbox->setValue(nav_action->y());
        d_->angle_spinbox->setValue(nav_action->theta());
        d_->tolerance_spinbox->setValue(nav_action->tolerance());
    }
    else if (auto* loop_action = dynamic_cast<LoopActionBlock*>(action)) {
        d_->stacked_widget->setCurrentWidget(d_->loop_page);
        d_->count_spinbox->setValue(loop_action->count());
        updateActionList(d_->action_combobox);
        int index = d_->action_combobox->findText(loop_action->action());
        if (index >= 0) {
            d_->action_combobox->setCurrentIndex(index);
        }
    }
    else if (auto* condition_action = dynamic_cast<ConditionActionBlock*>(action)) {
        d_->stacked_widget->setCurrentWidget(d_->condition_page);
        int type_index = d_->condition_type_combobox->findText(condition_action->condition());
        if (type_index >= 0) {
            d_->condition_type_combobox->setCurrentIndex(type_index);
        }
        d_->threshold_spinbox->setValue(condition_action->sensorThreshold());
        
        updateActionList(d_->true_action_combobox);
        updateActionList(d_->false_action_combobox);
        
        int true_index = d_->true_action_combobox->findText(condition_action->trueAction());
        if (true_index >= 0) {
            d_->true_action_combobox->setCurrentIndex(true_index);
        }
        
        int false_index = d_->false_action_combobox->findText(condition_action->falseAction());
        if (false_index >= 0) {
            d_->false_action_combobox->setCurrentIndex(false_index);
        }
    }
}

void SettingsWidget::updateSettings()
{
    if (!d_->current_action) return;
    
    if (auto* nav_action = dynamic_cast<NavigationActionBlock*>(d_->current_action)) {
        nav_action->setX(d_->x_spinbox->value());
        nav_action->setY(d_->y_spinbox->value());
        nav_action->setTheta(d_->angle_spinbox->value());
        nav_action->setTolerance(d_->tolerance_spinbox->value());
    }
    else if (auto* loop_action = dynamic_cast<LoopActionBlock*>(d_->current_action)) {
        loop_action->setCount(d_->count_spinbox->value());
        loop_action->setAction(d_->action_combobox->currentText());
    }
    else if (auto* condition_action = dynamic_cast<ConditionActionBlock*>(d_->current_action)) {
        condition_action->setCondition(d_->condition_type_combobox->currentText());
        condition_action->setSensorThreshold(d_->threshold_spinbox->value());
        condition_action->setTrueAction(d_->true_action_combobox->currentText());
        condition_action->setFalseAction(d_->false_action_combobox->currentText());
    }
    
    emit settingsUpdated();
}

void SettingsWidget::clearSettings()
{
    d_->current_action = nullptr;
    d_->x_spinbox->setValue(0.0);
    d_->y_spinbox->setValue(0.0);
    d_->angle_spinbox->setValue(0.0);
    d_->tolerance_spinbox->setValue(0.1);
    d_->count_spinbox->setValue(1);
    d_->threshold_spinbox->setValue(0.0);
    d_->action_combobox->clear();
    d_->true_action_combobox->clear();
    d_->false_action_combobox->clear();
}

void SettingsWidget::updateActionList(QComboBox* combobox)
{
    if (!combobox) return;
    
    combobox->clear();
    combobox->addItem(tr("导航动作"));
    combobox->addItem(tr("路径动作"));
    combobox->addItem(tr("移动动作"));
    combobox->addItem(tr("等待动作"));
    combobox->addItem(tr("循环动作"));
    combobox->addItem(tr("条件动作"));
} 