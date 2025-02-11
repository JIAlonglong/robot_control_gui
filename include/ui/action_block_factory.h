#pragma once

#ifndef ROBOT_CONTROL_GUI_ACTION_BLOCK_FACTORY_H
#define ROBOT_CONTROL_GUI_ACTION_BLOCK_FACTORY_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QIcon>
#include <memory>
#include <functional>

class RobotController;
class ActionBlock;

/**
 * @class ActionBlockFactory
 * @brief 动作块工厂类,负责创建和管理不同类型的动作块
 */
class ActionBlockFactory : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param controller 机器人控制器实例
     * @param parent 父对象
     */
    explicit ActionBlockFactory(const std::shared_ptr<RobotController>& controller, QObject* parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~ActionBlockFactory() override;

    /**
     * @brief 获取所有可用的动作类型
     * @return 动作类型列表
     */
    QStringList availableTypes() const;

    /**
     * @brief 创建指定类型的动作块
     * @param type 动作类型
     * @param parent 父对象
     * @return 创建的动作块实例
     */
    ActionBlock* create(const QString& type, QObject* parent = nullptr);

    /**
     * @brief 创建指定类型的动作块(不指定父对象)
     * @param type 动作类型
     * @return 创建的动作块实例
     */
    ActionBlock* createAction(const QString& type);

    /**
     * @brief 设置机器人控制器
     * @param controller 机器人控制器实例
     */
    void setRobotController(std::shared_ptr<RobotController> controller);

    /**
     * @brief 注册动作类型及其创建函数
     * @param type 动作类型
     * @param creator 创建函数
     */
    void registerActionType(const QString& type, std::function<ActionBlock*(QObject*)> creator);

    /**
     * @brief 获取动作名称
     * @param type 动作类型
     * @return 动作名称
     */
    QString actionName(const QString& type) const;

    /**
     * @brief 获取动作描述
     * @param type 动作类型
     * @return 动作描述
     */
    QString actionDescription(const QString& type) const;

    /**
     * @brief 获取动作图标
     * @param type 动作类型
     * @return 动作图标
     */
    QIcon actionIcon(const QString& type) const;

Q_SIGNALS:
    /**
     * @brief 当新的动作类型被注册时发出
     * @param type 新注册的动作类型
     */
    void actionTypeRegistered(const QString& type);

    /**
     * @brief 当创建动作块失败时发出
     * @param type 动作类型
     * @param error 错误信息
     */
    void creationFailed(const QString& type, const QString& error);

private:
    struct Private;
    std::unique_ptr<Private> d_;

    // 禁止拷贝和赋值
    ActionBlockFactory(const ActionBlockFactory&) = delete;
    ActionBlockFactory& operator=(const ActionBlockFactory&) = delete;
};

#endif // ROBOT_CONTROL_GUI_ACTION_BLOCK_FACTORY_H 