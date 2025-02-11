# 使用ROS Noetic基础镜像
FROM osrf/ros:noetic-desktop-full

# 设置工作目录
WORKDIR /root/catkin_ws

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    qt5-default \
    libqt5opengl5-dev \
    libqt5multimedia5 \
    libqt5multimedia5-plugins \
    libqt5multimediawidgets5 \
    libqt5serialport5-dev \
    ros-noetic-rviz \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-navigation \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    ros-noetic-global-planner \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# 创建catkin工作空间
RUN mkdir -p /root/catkin_ws/src

# 复制源代码
COPY . /root/catkin_ws/src/robot_control_gui/

# 设置环境变量
ENV TURTLEBOT3_MODEL=burger
ENV QT_X11_NO_MITSHM=1

# 编译代码
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make'

# 添加启动脚本
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "robot_control_gui", "robot_control_gui.launch"] 