#ifndef ACTION_SELECTION_DIALOG_H
#define ACTION_SELECTION_DIALOG_H

#include <QDialog>
#include <QListWidget>
#include <QStringList>
#include <memory>

class ActionSelectionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ActionSelectionDialog(const QStringList& types, QWidget* parent = nullptr);
    ~ActionSelectionDialog() override = default;

    QString selectedType() const;

private:
    struct Private {
        QListWidget* list{nullptr};
    };
    std::unique_ptr<Private> d_;
};

#endif  // ACTION_SELECTION_DIALOG_H