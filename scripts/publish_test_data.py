#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
import tf.transformations

def create_test_map():
    # 创建一个20x20米的地图，分辨率为0.05米
    width = 400  # 20米/0.05米
    height = 400
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = "map"
    map_msg.header.stamp = rospy.Time.now()
    
    # 设置地图属性
    map_msg.info.resolution = 0.05
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.origin.position.x = -10.0  # 地图中心在原点
    map_msg.info.origin.position.y = -10.0
    
    # 创建地图数据
    data = np.zeros(width * height, dtype=np.int8)
    
    # 添加边界墙
    for i in range(width):
        data[i] = 100  # 底边
        data[i + (height-1)*width] = 100  # 顶边
    for i in range(height):
        data[i*width] = 100  # 左边
        data[i*width + width-1] = 100  # 右边
    
    # 添加一些随机障碍物
    for _ in range(50):
        x = np.random.randint(50, width-50)
        y = np.random.randint(50, height-50)
        # 创建一个小方块障碍物
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                idx = (y+dy)*width + (x+dx)
                if 0 <= idx < len(data):
                    data[idx] = 100
    
    map_msg.data = list(data)
    return map_msg

def create_test_scan():
    scan_msg = LaserScan()
    scan_msg.header.frame_id = "base_laser"
    scan_msg.header.stamp = rospy.Time.now()
    
    # 设置激光扫描参数
    scan_msg.angle_min = -np.pi/2
    scan_msg.angle_max = np.pi/2
    scan_msg.angle_increment = np.pi/180  # 1度分辨率
    scan_msg.time_increment = 0.0
    scan_msg.range_min = 0.1
    scan_msg.range_max = 10.0
    
    # 生成180个扫描点
    num_readings = 180
    ranges = []
    for i in range(num_readings):
        # 生成一些随机的但有意义的距离数据
        base_range = 2.0  # 基础距离
        noise = np.random.normal(0, 0.1)  # 添加一些噪声
        range_val = base_range + noise
        ranges.append(max(0.1, min(range_val, 10.0)))  # 限制在有效范围内
    
    scan_msg.ranges = ranges
    return scan_msg

def main():
    rospy.init_node('test_data_publisher')
    
    # 创建发布器
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    
    rate = rospy.Rate(10)  # 10Hz
    
    # 发布地图（只需发布一次）
    map_msg = create_test_map()
    map_pub.publish(map_msg)
    
    # 机器人运动参数
    x = 0.0
    y = 0.0
    theta = 0.0
    
    while not rospy.is_shutdown():
        # 更新机器人位置（模拟圆周运动）
        time = rospy.Time.now().to_sec()
        x = 2.0 * np.cos(0.2 * time)
        y = 2.0 * np.sin(0.2 * time)
        theta = 0.2 * time
        
        # 创建和发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "base_link"
        
        # 设置位置
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # 设置方向（四元数）
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation = Quaternion(*q)
        
        # 发布消息
        odom_pub.publish(odom_msg)
        scan_pub.publish(create_test_scan())
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 