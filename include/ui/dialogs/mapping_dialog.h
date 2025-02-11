#ifndef MAPPING_DIALOG_H
#define MAPPING_DIALOG_H

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QString>

class MappingDialog : public QDialog
{
    Q_OBJECT

public:
    enum class MappingMethod { GMAPPING, CARTOGRAPHER, HECTOR_SLAM, RTAB_MAP };

    explicit MappingDialog(QWidget* parent = nullptr);
    ~MappingDialog() = default;

    MappingMethod getSelectedMethod() const;
    QString       getMapName() const;
    QString       getMapPath() const;
    double        getResolution() const;
    bool          getUpdateInRealTime() const;
    int           getParticleCount() const;  // For Gmapping
    double        getMinimumScore() const;   // For Gmapping
    double        getMaximumRange() const;   // For laser settings
    bool          getUseOdom() const;        // For Hector SLAM

private Q_SLOTS:
    void onMethodChanged(int index);
    void onBrowsePath();
    void validateInput();

private:
    void setupUi();
    void setupConnections();
    void updateParameterVisibility();

    QComboBox*      method_combo_{nullptr};
    QLineEdit*      map_name_edit_{nullptr};
    QLineEdit*      map_path_edit_{nullptr};
    QPushButton*    browse_button_{nullptr};
    QDoubleSpinBox* resolution_spin_{nullptr};
    QCheckBox*      real_time_check_{nullptr};

    // Gmapping specific parameters
    QWidget*        gmapping_widget_{nullptr};
    QSpinBox*       particle_count_spin_{nullptr};
    QDoubleSpinBox* minimum_score_spin_{nullptr};

    // Common laser parameters
    QDoubleSpinBox* max_range_spin_{nullptr};

    // Hector SLAM specific parameters
    QWidget*   hector_widget_{nullptr};
    QCheckBox* use_odom_check_{nullptr};

    QPushButton* ok_button_{nullptr};
    QPushButton* cancel_button_{nullptr};
};

#endif  // MAPPING_DIALOG_H