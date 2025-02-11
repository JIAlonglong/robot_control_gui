#pragma once

#ifndef ROBOT_CONTROL_GUI_MAPPING_PANEL_H
#define ROBOT_CONTROL_GUI_MAPPING_PANEL_H

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include <memory>

#include "mapping_controller.h"
#include "robot_controller.h"
#include "rviz_view.h"

class MappingPanel : public QWidget
{
    Q_OBJECT

public:
    explicit MappingPanel(QWidget* parent = nullptr);
    explicit MappingPanel(std::shared_ptr<MappingController> controller, QWidget* parent = nullptr);
    ~MappingPanel();

    void setMappingController(const std::shared_ptr<MappingController>& controller);

private Q_SLOTS:
    void onStartMapping();
    void onStopMapping();
    void onSaveMap();
    void onStartAutoMapping();
    void onStopAutoMapping();
    void onPauseMapping();
    void onResumeMapping();
    void onMethodChanged(const QString& method);
    void onResolutionChanged(double value);
    void onUpdateRateChanged(int value);
    void onStrategyChanged(const QString& strategy);
    void onRadiusChanged(double value);
    void onFrontierSizeChanged(double value);
    void onMaxTimeChanged(int value);
    void onReturnHomeChanged(int state);
    void onRealTimeUpdateChanged(int state);
    void onMappingStateChanged(const QString& state);
    void onMappingProgressChanged(double progress);
    void onMappingFailed(const QString& error);
    void onMappingFinished();
    void onMapSaved(const QString& filename);
    void onExplorationProgressChanged(double progress);
    void onReturnHomeStarted();
    void onReturnHomeCompleted();
    void onExplorationCompleted();
    void onExplorationFailed(const QString& error);
    void onMappingStarted();
    void onMappingCancelled();
    void onMappingStatusUpdated(const QString& status);
    void onMappingProgressUpdated(double progress);
    void onExplorationProgressUpdated(double progress);
    void updateMappingStatus(const QString& status);
    void updateMappingProgress(double progress);
    void updateButtonStates();

private:
    void setupUi();
    void setupAutoMappingGroup();
    void connectSignals();
    void updateButtons();

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif  // ROBOT_CONTROL_GUI_MAPPING_PANEL_H