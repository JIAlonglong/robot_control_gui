/**
 * @file robot_controller.h
 * @brief 机器人控制器基类的声明
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <QObject>
#include <string>

/**
 * @brief 机器人控制器基类
 * 
 * 定义了所有机器人控制器必须实现的基本接口
 */
class RobotController : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param robot_name 机器人名称
     */
    explicit RobotController(const std::string& robot_name);

    /**
     * @brief 移动机器人
     * @param linear_x 线速度
     * @param angular_z 角速度
     */
    virtual void move(double linear_x, double angular_z) = 0;

    /**
     * @brief 停止机器人
     */
    virtual void stop() = 0;

signals:
    /**
     * @brief 机器人状态更新信号
     * @param status 状态信息
     */
    void statusUpdated(const QString& status);

protected:
    std::string robot_name_;
};

#endif // ROBOT_CONTROLLER_H 