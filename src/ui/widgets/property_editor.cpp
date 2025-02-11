/**
 * Copyright (c) 2024 JIAlonglong
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
 * @file property_editor.cpp
 * @brief 属性编辑器组件的实现,用于编辑动作块的属性
 * @author JIAlonglong
 */

#include "property_editor.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>
#include "action_block.h"

struct PropertyEditor::Private {
    ActionBlock*            action{nullptr};
    QFormLayout*            form_layout{nullptr};
    QMap<QString, QWidget*> editors;
};

PropertyEditor::PropertyEditor(QWidget* parent) : QWidget(parent), d_(std::make_unique<Private>())
{
    setupUi();
}

PropertyEditor::~PropertyEditor() = default;

void PropertyEditor::setActionBlock(ActionBlock* action)
{
    if (d_->action == action) return;

    if (d_->action) {
        disconnect(d_->action, nullptr, this, nullptr);
    }

    d_->action = action;

    if (d_->action) {
        connect(d_->action, &ActionBlock::propertyChanged, this,
                &PropertyEditor::onActionPropertyChanged);
    }

    clearPropertyWidgets();
    createPropertyWidgets();
}

ActionBlock* PropertyEditor::actionBlock() const
{
    return d_->action;
}

void PropertyEditor::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    d_->form_layout = new QFormLayout;
    d_->form_layout->setContentsMargins(4, 4, 4, 4);
    d_->form_layout->setSpacing(4);
    layout->addLayout(d_->form_layout);

    layout->addStretch();
}

void PropertyEditor::createPropertyWidgets()
{
    if (!d_->action) return;

    const auto properties = d_->action->properties();
    for (auto it = properties.begin(); it != properties.end(); ++it) {
        const QString&  name  = it.key();
        const QVariant& value = it.value();

        auto* widget = createPropertyWidget(name, value);
        if (widget) {
            d_->editors[name] = widget;
            d_->form_layout->addRow(tr(name.toUtf8()), widget);
        }
    }
}

void PropertyEditor::clearPropertyWidgets()
{
    qDeleteAll(d_->editors);
    d_->editors.clear();

    while (d_->form_layout->count() > 0) {
        auto* item = d_->form_layout->takeAt(0);
        delete item->widget();
        delete item;
    }
}

QWidget* PropertyEditor::createPropertyWidget(const QString& name, const QVariant& value)
{
    switch (value.type()) {
        case QVariant::String: {
            auto* editor = new QLineEdit(this);
            editor->setText(value.toString());
            connect(editor, &QLineEdit::textChanged, this,
                    [this, name](const QString& text) { onPropertyValueChanged(name, text); });
            return editor;
        }

        case QVariant::Int: {
            auto* editor = new QSpinBox(this);
            editor->setValue(value.toInt());
            editor->setRange(std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
            connect(editor, QOverload<int>::of(&QSpinBox::valueChanged), this,
                    [this, name](int value) { onPropertyValueChanged(name, value); });
            return editor;
        }

        case QVariant::Double: {
            auto* editor = new QDoubleSpinBox(this);
            editor->setValue(value.toDouble());
            editor->setRange(-1000000.0, 1000000.0);
            editor->setDecimals(3);
            connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                    [this, name](double value) { onPropertyValueChanged(name, value); });
            return editor;
        }

        case QVariant::Bool: {
            auto* editor = new QCheckBox(this);
            editor->setChecked(value.toBool());
            connect(editor, &QCheckBox::toggled, this,
                    [this, name](bool checked) { onPropertyValueChanged(name, checked); });
            return editor;
        }

        case QVariant::StringList: {
            auto* editor = new QComboBox(this);
            editor->addItems(value.toStringList());
            connect(editor, &QComboBox::currentTextChanged, this,
                    [this, name](const QString& text) { onPropertyValueChanged(name, text); });
            return editor;
        }

        default:
            qWarning() << "不支持的属性类型:" << value.type();
            return nullptr;
    }
}

void PropertyEditor::onPropertyValueChanged(const QString& name, const QVariant& value)
{
    if (d_->action) {
        d_->action->setProperty(name.toUtf8(), value);
        emit propertyChanged(name, value);
    }
}

void PropertyEditor::onActionPropertyChanged(const QString& name, const QVariant& value)
{
    auto it = d_->editors.find(name);
    if (it == d_->editors.end()) return;

    auto* widget = it.value();
    if (!widget) return;

    if (auto* line_edit = qobject_cast<QLineEdit*>(widget)) {
        line_edit->setText(value.toString());
    } else if (auto* spin_box = qobject_cast<QSpinBox*>(widget)) {
        spin_box->setValue(value.toInt());
    } else if (auto* double_spin_box = qobject_cast<QDoubleSpinBox*>(widget)) {
        double_spin_box->setValue(value.toDouble());
    } else if (auto* check_box = qobject_cast<QCheckBox*>(widget)) {
        check_box->setChecked(value.toBool());
    } else if (auto* combo_box = qobject_cast<QComboBox*>(widget)) {
        combo_box->setCurrentText(value.toString());
    }
}

#include "property_editor.moc"