#include "ui/action_selection_dialog.h"
#include <QVBoxLayout>
#include <QListWidget>
#include <QDialogButtonBox>

ActionSelectionDialog::ActionSelectionDialog(const QStringList& types, QWidget* parent)
    : QDialog(parent)
    , d_(std::make_unique<Private>())
{
    setWindowTitle(tr("选择动作类型"));
    setModal(true);

    auto* layout = new QVBoxLayout(this);

    d_->list = new QListWidget(this);
    d_->list->addItems(types);
    d_->list->setCurrentRow(0);
    layout->addWidget(d_->list);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        Qt::Horizontal,
        this
    );
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

QString ActionSelectionDialog::selectedType() const
{
    QListWidgetItem* item = d_->list->currentItem();
    return item ? item->text() : QString();
}