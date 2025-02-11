#!/bin/bash
set -e

# 设置ROS环境
source "/opt/ros/noetic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

# 启动roscore(后台运行)
roscore &
sleep 2

# 启动仿真环境(如果需要)
if [ "$LAUNCH_SIMULATION" = "true" ]; then
    echo "启动TurtleBot3仿真环境..."
    roslaunch turtlebot3_gazebo turtlebot3_world.launch &
    sleep 5
fi

# 执行传入的命令
exec "$@" 