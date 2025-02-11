#ifndef PROPERTY_EDITOR_H
#define PROPERTY_EDITOR_H

#include <QWidget>
#include <memory>
#include <QVariant>

class ActionBlock;

class PropertyEditor : public QWidget {
    Q_OBJECT

public:
    explicit PropertyEditor(QWidget* parent = nullptr);
    ~PropertyEditor() override;

    void setActionBlock(ActionBlock* action);
    ActionBlock* actionBlock() const;

Q_SIGNALS:
    void propertyChanged(const QString& name, const QVariant& value);

private Q_SLOTS:
    void onPropertyValueChanged(const QString& name, const QVariant& value);
    void onActionPropertyChanged(const QString& name, const QVariant& value);

private:
    void setupUi();
    void createPropertyWidgets();
    void clearPropertyWidgets();
    QWidget* createPropertyWidget(const QString& name, const QVariant& value);

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif // PROPERTY_EDITOR_H 