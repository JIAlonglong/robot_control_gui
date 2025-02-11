#pragma once

#ifndef SETTINGS_PANEL_H
#define SETTINGS_PANEL_H

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>
#include <memory>

class RobotController;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QVBoxLayout;

class SettingsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit SettingsPanel(QWidget* parent = nullptr);
    ~SettingsPanel() override;

    void setRobotController(const std::shared_ptr<RobotController>& controller);

Q_SIGNALS:
    void settingsChanged();

private Q_SLOTS:
    void onAutoConnectChanged(bool enabled);
    void onDebugModeChanged(bool enabled);
    void onRealTimeUpdateChanged(bool enabled);
    void onSaveSettings();
    void onRestoreSettings();

private:
    void setupUi();
    void updateSettings();
    void loadSettings();
    void saveSettings();

    struct Private {
        QVBoxLayout*                     layout{nullptr};
        QCheckBox*                       auto_connect_checkbox{nullptr};
        QCheckBox*                       debug_mode_checkbox{nullptr};
        QCheckBox*                       real_time_update_checkbox{nullptr};
        QPushButton*                     save_button{nullptr};
        QPushButton*                     restore_button{nullptr};
        std::shared_ptr<RobotController> robot_controller;
    };
    std::unique_ptr<Private> d_;
};

#endif  // SETTINGS_PANEL_H