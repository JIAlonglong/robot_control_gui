#ifndef SETTINGS_WIDGET_H
#define SETTINGS_WIDGET_H

#include <QWidget>
#include <memory>

class ActionBlock;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QVBoxLayout;
class QStackedWidget;
class QSpinBox;

class SettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SettingsWidget(QWidget* parent = nullptr);
    ~SettingsWidget() override;

    void setActionBlock(ActionBlock* action);

Q_SIGNALS:
    void settingsUpdated();

private Q_SLOTS:
    void updateSettings();
    void updateActionList(QComboBox* combobox);
    void clearSettings();

private:
    void setupUi();
    void setupNavigationSettings();
    void setupLoopSettings();
    void setupConditionSettings();
    void clearLayout(QLayout* layout);

    struct Private {
        QVBoxLayout* layout{nullptr};
        QStackedWidget* stacked_widget{nullptr};
        
        // Navigation settings
        QWidget* navigation_page{nullptr};
        QDoubleSpinBox* x_spinbox{nullptr};
        QDoubleSpinBox* y_spinbox{nullptr};
        QDoubleSpinBox* angle_spinbox{nullptr};
        QDoubleSpinBox* tolerance_spinbox{nullptr};

        // Loop settings
        QWidget* loop_page{nullptr};
        QComboBox* action_combobox{nullptr};
        QSpinBox* count_spinbox{nullptr};

        // Condition settings
        QWidget* condition_page{nullptr};
        QComboBox* condition_type_combobox{nullptr};
        QDoubleSpinBox* threshold_spinbox{nullptr};
        QComboBox* true_action_combobox{nullptr};
        QComboBox* false_action_combobox{nullptr};

        ActionBlock* current_action{nullptr};
    };
    std::unique_ptr<Private> d_;
};

#endif // SETTINGS_WIDGET_H 