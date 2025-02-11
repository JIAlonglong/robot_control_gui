/**
 * Copyright (c) 2025 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file action_selection_dialog.cpp
 * @brief 未知组件
 * @author JIAlonglong
 */

#include "action_selection_dialog.h"
#include <QDialogButtonBox>
#include <QListWidget>
#include <QVBoxLayout>

ActionSelectionDialog::ActionSelectionDialog(const QStringList& types, QWidget* parent)
    : QDialog(parent), d_(std::make_unique<Private>())
{
    setWindowTitle(tr("选择动作类型"));
    setModal(true);

    auto* layout = new QVBoxLayout(this);

    d_->list = new QListWidget(this);
    d_->list->addItems(types);
    d_->list->setCurrentRow(0);
    layout->addWidget(d_->list);

    auto* buttons =
        new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

QString ActionSelectionDialog::selectedType() const
{
    QListWidgetItem* item = d_->list->currentItem();
    return item ? item->text() : QString();
}