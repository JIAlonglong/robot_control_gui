#ifndef DELAY_ACTION_BLOCK_H
#define DELAY_ACTION_BLOCK_H

#include <QTimer>
#include "action_block.h"

class DelayActionBlock : public ActionBlock
{
    Q_OBJECT
    Q_PROPERTY(double duration READ duration WRITE setDuration NOTIFY durationChanged)

public:
    explicit DelayActionBlock(QObject* parent = nullptr);
    ~DelayActionBlock() override;

    QString type() const override
    {
        return "延时动作";
    }
    QString name() const override
    {
        return tr("延时动作");
    }
    QString description() const override
    {
        return tr("等待指定时间");
    }
    QIcon icon() const override;
    bool  isRunning() const override
    {
        return is_running_;
    }
    void reset() override;

    double duration() const
    {
        return duration_;
    }
    void setDuration(double duration);

    QVariantMap properties() const override;
    void        setProperty(const QString& name, const QVariant& value) override;

    bool isValid() const override
    {
        return duration_ > 0.0;
    }

public Q_SLOTS:
    void execute() override;
    void stop() override;

Q_SIGNALS:
    void durationChanged(double duration);

private Q_SLOTS:
    void onTimeout();

private:
    double duration_{1.0};
    bool   is_running_{false};
    QTimer timer_;
};

#endif  // DELAY_ACTION_BLOCK_H