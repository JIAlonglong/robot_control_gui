#pragma once

#include <QTimer>
#include "action_block.h"

class LoopActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(int count READ count WRITE setCount NOTIFY countChanged)
    Q_PROPERTY(ActionBlock* action READ action WRITE setAction NOTIFY actionChanged)

public:
    explicit LoopActionBlock(QObject* parent = nullptr);
    ~LoopActionBlock() override;

    QString type() const override;
    QString name() const override;
    QString description() const override;
    QIcon   icon() const override;
    bool    isValid() const override;
    bool    isRunning() const override;

    int  count() const;
    void setCount(int count);

    ActionBlock* action() const;
    void         setAction(ActionBlock* action);

    QVariantMap properties() const override;
    void        setProperty(const QString& name, const QVariant& value) override;
    QJsonObject toJson() const override;
    void        fromJson(const QJsonObject& json) override;

public Q_SLOTS:
    void execute() override;
    void stop() override;
    void reset() override;
    void pause() override;
    void resume() override;

Q_SIGNALS:
    void countChanged(int count);
    void actionChanged(ActionBlock* action);

private Q_SLOTS:
    void onActionCompleted(bool success);

private:
    struct Private;
    std::unique_ptr<Private> d_;
};