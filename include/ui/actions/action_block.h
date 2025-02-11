#pragma once

#ifndef ROBOT_CONTROL_GUI_ACTION_BLOCK_H
#define ROBOT_CONTROL_GUI_ACTION_BLOCK_H

#include <QIcon>
#include <QJsonObject>
#include <QMap>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QVariant>
#include <memory>

class ActionBlock : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString type READ type CONSTANT)
    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(QString description READ description CONSTANT)
    Q_PROPERTY(QIcon icon READ icon CONSTANT)
    Q_PROPERTY(bool isRunning READ isRunning NOTIFY runningChanged)
    Q_PROPERTY(QVariantMap properties READ properties NOTIFY propertiesChanged)
    Q_PROPERTY(QString category READ category WRITE setCategory NOTIFY categoryChanged)
    Q_PROPERTY(QPoint position READ position WRITE setPosition NOTIFY positionChanged)
    Q_PROPERTY(QPoint inputPoint READ inputPoint WRITE setInputPoint NOTIFY inputPointChanged)
    Q_PROPERTY(QPoint outputPoint READ outputPoint WRITE setOutputPoint NOTIFY outputPointChanged)

public:
    explicit ActionBlock(QObject* parent = nullptr);
    ~ActionBlock() override;

    virtual QString     type() const = 0;
    virtual QString     name() const = 0;
    virtual QString     description() const = 0;
    virtual QIcon       icon() const = 0;
    virtual bool        isValid() const = 0;
    virtual bool        isRunning() const;
    virtual QVariantMap properties() const = 0;
    virtual void        setProperty(const QString& name, const QVariant& value) = 0;
    virtual QJsonObject toJson() const;
    virtual void        fromJson(const QJsonObject& json);

    QString category() const;
    void    setCategory(const QString& category);

    virtual bool validateParameters() const;

    QPoint position() const;
    void   setPosition(const QPoint& pos);

    QVariant                parameter(const QString& key) const;
    void                    setParameter(const QString& key, const QVariant& value);
    QMap<QString, QVariant> parameters() const;
    void                    setParameters(const QMap<QString, QVariant>& params);

    QPoint inputPoint() const;
    QPoint outputPoint() const;
    void   setInputPoint(const QPoint& point);
    void   setOutputPoint(const QPoint& point);

    QVariantMap toVariantMap() const;
    void        fromVariantMap(const QVariantMap& map);

    void validateConnection(ActionBlock* target);

public Q_SLOTS:
    virtual void execute() = 0;
    virtual void stop();
    virtual void reset();
    virtual void pause();
    virtual void resume();

Q_SIGNALS:
    void started();
    void stopped();
    void paused();
    void resumed();
    void completed(bool success);
    void error(const QString& message);
    void propertyChanged(const QString& name, const QVariant& value);
    void parameterChanged(const QString& name, const QVariant& value);
    void progressChanged(double progress);
    void statusChanged(const QString& status);
    void runningChanged(bool running);
    void propertiesChanged();
    void categoryChanged(const QString& category);
    void positionChanged(const QPoint& pos);
    void inputPointChanged(const QPoint& point);
    void outputPointChanged(const QPoint& point);

protected:
    virtual void onExecutionStarted();
    virtual void onExecutionFinished(bool success);
    virtual void onExecutionError(const QString& error_msg);
    virtual void onExecutionStopped();
    virtual void onExecutionPaused();
    virtual void onExecutionResumed();

private:
    struct Private {
        QString                 category;
        bool                    is_running{false};
        bool                    is_paused{false};
        QPoint                  position;
        QPoint                  input_point;
        QPoint                  output_point;
        QMap<QString, QVariant> parameters;
    };
    std::unique_ptr<Private> d_;
};

#endif  // ROBOT_CONTROL_GUI_ACTION_BLOCK_H