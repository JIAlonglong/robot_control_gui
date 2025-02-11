#pragma once

#include <QTimer>
#include "action_block.h"

class WaitActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(double duration READ duration WRITE setDuration NOTIFY durationChanged)

public:
    explicit WaitActionBlock(QObject* parent = nullptr);
    ~WaitActionBlock() override;

    double duration() const;
    void   setDuration(double duration);

    QVariantMap properties() const override;
    void        setProperty(const QString& name, const QVariant& value) override;

public Q_SLOTS:
    void execute() override;
    void stop() override;

Q_SIGNALS:
    void durationChanged(double duration);

private Q_SLOTS:
    void onTimeout();

private:
    struct Private;
    std::unique_ptr<Private> d_;
};