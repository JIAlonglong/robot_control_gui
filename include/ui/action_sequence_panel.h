#pragma once

#ifndef ACTION_SEQUENCE_PANEL_H
#define ACTION_SEQUENCE_PANEL_H

#include <QWidget>
#include <QList>
#include <memory>

class ActionBlock;
class ActionBlockFactory;
class ActionConfigurator;
class ActionListWidget;
class ActionSequenceManager;
class QComboBox;
class QLabel;
class QPushButton;
class QStackedWidget;
class QVBoxLayout;

class ActionSequencePanel : public QWidget {
    Q_OBJECT

public:
    explicit ActionSequencePanel(QWidget* parent = nullptr);
    ~ActionSequencePanel() override;

    void setActionBlockFactory(const std::shared_ptr<ActionBlockFactory>& factory);
    void setActionSequenceManager(ActionSequenceManager* manager);
    void load(const QString& filename);
    void save(const QString& filename);

public Q_SLOTS:
    void execute();
    void stop();
    void pause();
    void resume();
    void clear();

Q_SIGNALS:
    void actionAdded(int index);
    void actionRemoved(int index);
    void actionMoved(int from, int to);
    void actionStarted(ActionBlock* action);
    void actionStopped(ActionBlock* action);
    void actionPaused(ActionBlock* action);
    void actionResumed(ActionBlock* action);
    void actionCompleted(ActionBlock* action);
    void actionFailed(ActionBlock* action, const QString& error);
    void sequenceStarted();
    void sequenceStopped();
    void sequencePaused();
    void sequenceResumed();
    void sequenceCompleted();
    void sequenceFailed(const QString& error);

private Q_SLOTS:
    void onAddAction();
    void onRemoveAction();
    void onMoveUpAction();
    void onMoveDownAction();
    void onActionSelected(int index);
    void onActionCompleted(ActionBlock* action);
    void onActionFailed(ActionBlock* action, const QString& error);
    void onSequenceStarted();
    void onSequenceStopped();
    void onSequenceCompleted();
    void onSequenceFailed(const QString& error);

private:
    void setupUi();
    void updateButtons();
    void connectAction(ActionBlock* action);
    void disconnectAction(ActionBlock* action);
    void executeNext();

    struct Private {
        QVBoxLayout* layout{nullptr};
        ActionListWidget* action_list{nullptr};
        ActionConfigurator* configurator{nullptr};
        QWidget* button_widget{nullptr};
        QPushButton* add_button{nullptr};
        QPushButton* remove_button{nullptr};
        QPushButton* up_button{nullptr};
        QPushButton* down_button{nullptr};
        QPushButton* start_button{nullptr};
        QPushButton* stop_button{nullptr};
        QPushButton* pause_button{nullptr};
        QList<ActionBlock*> actions;
        std::shared_ptr<ActionBlockFactory> factory;
        ActionBlock* current_action{nullptr};
        int current_action_index{-1};
        bool is_running{false};
        bool is_paused{false};
    };
    std::unique_ptr<Private> d_;
};

#endif // ACTION_SEQUENCE_PANEL_H 