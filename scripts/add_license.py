#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from datetime import datetime

MIT_LICENSE_TEMPLATE = '''/**
 * Copyright (c) {year} JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file {filename}
 * @brief {description}
 * @author JIAlonglong
 */

'''

FILE_DESCRIPTIONS = {
    # UI组件
    'battery_indicator': '电池指示器组件,用于显示机器人电池状态',
    'connection_line': '连接线组件,用于在动作编辑器中显示动作块之间的连接关系',
    'dashboard_window': '仪表盘窗口,用于显示机器人的状态信息',
    'camera_view': '相机视图组件,用于显示机器人摄像头画面',
    'joystick_widget': '虚拟摇杆组件,用于通过鼠标控制机器人移动',
    'speed_display': '速度显示组件,用于显示机器人的线速度和角速度',
    'speed_dashboard': '速度仪表盘组件,用于图形化显示机器人速度',
    'action_block_widget': '动作块组件,用于在动作编辑器中显示和编辑单个动作',
    'action_list_widget': '动作列表组件,用于显示和管理动作序列',
    'path_editor': '路径编辑器组件,用于编辑机器人导航路径',
    'path_visualizer': '路径可视化组件,用于显示机器人导航路径',
    'property_editor': '属性编辑器组件,用于编辑动作块的属性',
    'rviz_view': 'RViz可视化组件,用于3D显示机器人状态',
    
    # 主要类
    'main_window': '主窗口类,管理整个应用程序的界面',
    'main_window_private': '主窗口私有实现类,处理主窗口的内部逻辑',
    
    # 控制器类
    'robot_controller': '机器人控制器类,负责与ROS通信控制机器人',
    'mapping_controller': '建图控制器类,负责机器人建图功能',
    'navigation_controller': '导航控制器类,负责机器人导航功能',
    'action_controller': '动作控制器类,负责执行机器人动作序列',
    
    # 面板类
    'navigation_panel': '导航面板类,提供导航相关功能',
    'mapping_panel': '建图面板类,提供建图相关功能',
    'control_panel': '控制面板类,提供基本控制功能',
    'settings_panel': '设置面板类,提供参数配置功能',
    'action_sequence_panel': '动作序列面板类,用于编辑和执行动作序列',
    'teleoperation_panel': '遥操作面板类,提供远程控制功能',
    
    # 动作相关
    'action_configurator': '动作配置器类,用于配置机器人动作',
    'action_block': '动作块基类,定义动作的基本接口',
    'action_block_factory': '动作块工厂类,负责创建各种类型的动作块',
    'action_sequence_manager': '动作序列管理器,负责管理和执行动作序列',
    
    # ROS通信
    'ros_bridge': 'ROS桥接类,处理与ROS系统的通信',
    'ros_topic_manager': 'ROS话题管理器,管理ROS话题的订阅和发布',
    'ros_service_client': 'ROS服务客户端,处理ROS服务调用',
    'ros_action_client': 'ROS动作客户端,处理ROS动作调用',
    
    # 工具类
    'config_manager': '配置管理器,负责读写配置文件',
    'logger': '日志管理器,处理应用程序日志',
    'utils': '工具函数集合',
    'type_converter': '类型转换器,处理不同数据类型间的转换',
    
    # 传感器相关
    'laser_scan_handler': '激光扫描处理器,处理激光雷达数据',
    'imu_handler': 'IMU处理器,处理惯性测量单元数据',
    'odometry_handler': '里程计处理器,处理机器人位置信息',
    
    # 其他组件
    'waypoint_manager': '路径点管理器,管理导航路径点',
    'pose_manager': '位姿管理器,处理机器人位姿信息',
    'map_manager': '地图管理器,处理地图数据',
    'sensor_manager': '传感器管理器,管理各类传感器',
}

def get_file_description(filename):
    """根据文件名获取描述"""
    base_name = os.path.splitext(os.path.basename(filename))[0]
    for key, desc in FILE_DESCRIPTIONS.items():
        if key in base_name:
            return desc
    return '未知组件'

def has_license(content):
    """检查文件是否已有许可证头"""
    return 'Copyright (c)' in content[:500]

def add_license_to_file(filepath):
    """为单个文件添加许可证头"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
            
        if has_license(content):
            print(f"跳过 {filepath} - 已有许可证头")
            return False
            
        filename = os.path.basename(filepath)
        description = get_file_description(filename)
        license_text = MIT_LICENSE_TEMPLATE.format(
            year=datetime.now().year,
            filename=filename,
            description=description
        )
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(license_text + content)
            
        print(f"已添加许可证头到 {filepath}")
        return True
        
    except Exception as e:
        print(f"处理 {filepath} 时出错: {str(e)}")
        return False

def process_directory(directory):
    """处理目录下的所有源文件"""
    modified_files = 0
    total_files = 0
    
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.h')):
                total_files += 1
                filepath = os.path.join(root, file)
                if add_license_to_file(filepath):
                    modified_files += 1
                    
    return total_files, modified_files

def main():
    if len(sys.argv) < 2:
        print("用法: python add_license.py <源代码目录>")
        sys.exit(1)
        
    src_dir = sys.argv[1]
    if not os.path.isdir(src_dir):
        print(f"错误: {src_dir} 不是有效目录")
        sys.exit(1)
        
    print(f"开始处理目录: {src_dir}")
    total, modified = process_directory(src_dir)
    print(f"\n处理完成!")
    print(f"总文件数: {total}")
    print(f"已修改文件数: {modified}")
    print(f"跳过文件数: {total - modified}")

if __name__ == '__main__':
    main() 