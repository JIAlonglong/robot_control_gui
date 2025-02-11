#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys
from typing import Dict, List, Set, Tuple
import graphviz
from datetime import datetime

class SignalFlowAnalyzer:
    def __init__(self):
        self.signals = {}  # 类名 -> 信号列表
        self.slots = {}    # 类名 -> 槽列表
        self.connections = []  # (源类,信号,目标类,槽)
        self.emits = {}    # 类名 -> 发射的信号列表
        self.properties = {}  # 类名 -> 属性列表
        self.class_inheritance = {}  # 类名 -> 父类列表
        self.classes = {}  # 类名 -> 类信息
        self.inheritance = {}  # 类名 -> 父类

    def analyze_file(self, file_path: str) -> None:
        """分析单个文件中的信号和槽"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 提取类名和继承关系
        class_pattern = r'class\s+(\w+)\s*(?::\s*(?:public|private|protected)?\s*(\w+))?'
        for match in re.finditer(class_pattern, content):
            current_class = match.group(1)
            parent_class = match.group(2)
            if current_class not in self.classes:
                self.classes[current_class] = {'signals': set(), 'slots': set(), 'properties': set(), 'emits': set(), 'connects': []}
            if parent_class:
                self.inheritance[current_class] = parent_class

            # 如果不是QObject派生类，跳过
            if 'Q_OBJECT' not in content:
                continue

            # 查找Q_PROPERTY定义和NOTIFY信号
            prop_pattern = r'Q_PROPERTY\s*\(\s*\w+\s+(\w+)[^)]*NOTIFY\s+(\w+)[^)]*\)'
            for prop_match in re.finditer(prop_pattern, content):
                _, notify_signal = prop_match.groups()
                if current_class not in self.signals:
                    self.signals[current_class] = []
                self.signals[current_class].append(notify_signal)

            # 查找信号定义
            signal_patterns = [
                r'Q_SIGNAL\s+(?:public|private|protected)?\s*:?\s*(?:virtual)?\s*void\s+(\w+)',
                r'signals\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'Q_SIGNALS\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'emit\s+(\w+)',
                r'Q_EMIT\s+(\w+)'
            ]
            
            for pattern in signal_patterns:
                for match in re.finditer(pattern, content):
                    signal_name = match.group(1)
                    if current_class in self.classes:
                        self.classes[current_class]['signals'].add(signal_name)

            # 查找槽定义
            slot_patterns = [
                r'Q_SLOT\s+(?:public|private|protected)?\s*:?\s*(?:virtual)?\s*void\s+(\w+)',
                r'slots\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'Q_SLOTS\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'public slots\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'private slots\s*:(?:[^}]*?)\s+void\s+(\w+)',
                r'protected slots\s*:(?:[^}]*?)\s+void\s+(\w+)'
            ]
            
            for pattern in slot_patterns:
                for match in re.finditer(pattern, content):
                    slot_name = match.group(1)
                    if current_class in self.classes:
                        self.classes[current_class]['slots'].add(slot_name)

            # 查找emit语句
            emit_patterns = [
                r'emit\s+(\w+)\s*\(',
                r'Q_EMIT\s+(\w+)\s*\(',
                r'Q_EMIT\s+(\w+)\s*\(',
                r'emit\s+(\w+)\s*\('
            ]
            
            for pattern in emit_patterns:
                for match in re.finditer(pattern, content):
                    signal_name = match.group(1)
                    if current_class in self.classes:
                        self.classes[current_class]['emits'].add(signal_name)

            # 查找connect语句
            connect_patterns = [
                # 新式语法
                r'connect\s*\(\s*(?:this|&?\w+)\s*,\s*&\s*(\w+)::(\w+)\s*,\s*(?:this|&?\w+)\s*,\s*&\s*(\w+)::(\w+)\s*\)',
                # 旧式语法
                r'connect\s*\(\s*(?:this|&?\w+)\s*,\s*SIGNAL\s*\(\s*(\w+)[^)]*\)\s*,\s*(?:this|&?\w+)\s*,\s*SLOT\s*\(\s*(\w+)[^)]*\)\s*\)',
                # Lambda语法
                r'connect\s*\(\s*(?:this|&?\w+)\s*,\s*&\s*(\w+)::(\w+)\s*,\s*\[([^]]+)\]\s*\([^{]*\)\s*{([^}]*)}\s*\)',
                # 成员函数指针
                r'connect\s*\(\s*(?:this|&?\w+)\s*,\s*&\s*(\w+)::(\w+)\s*,\s*(?:this|&?\w+)\s*,\s*&\s*(\w+)::(\w+)\s*\)'
            ]

            for pattern in connect_patterns:
                for match in re.finditer(pattern, content):
                    groups = match.groups()
                    if len(groups) >= 2:
                        if 'SIGNAL' in pattern:
                            sender = current_class
                            signal = groups[0]
                            receiver = current_class
                            slot = groups[1]
                        elif '[' in pattern:
                            sender = groups[0]
                            signal = groups[1]
                            receiver = current_class
                            slot = 'lambda'
                        else:
                            sender = groups[0]
                            signal = groups[1]
                            receiver = groups[2] if len(groups) > 2 else current_class
                            slot = groups[3] if len(groups) > 3 else 'lambda'

                        # 清理变量名
                        sender = re.sub(r'[^a-zA-Z0-9_]', '', sender)
                        receiver = re.sub(r'[^a-zA-Z0-9_]', '', receiver)
                        signal = re.sub(r'[^a-zA-Z0-9_]', '', signal)
                        slot = re.sub(r'[^a-zA-Z0-9_]', '', slot)

                        if current_class in self.classes:
                            self.classes[current_class]['connects'].append({
                                'sender': sender,
                                'signal': signal,
                                'receiver': receiver,
                                'slot': slot
                            })

    def analyze_directory(self, directory: str) -> None:
        """分析目录下所有C++源文件"""
        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith(('.cpp', '.h')):
                    file_path = os.path.join(root, file)
                    self.analyze_file(file_path)

    def generate_graph(self, output_file: str = 'signal_flow') -> None:
        dot = graphviz.Digraph(comment='Signal Flow Graph')
        dot.attr(rankdir='TB', splines='ortho')

        # 添加标题
        dot.attr(label='''Robot Control GUI\nSignal Flow Diagram\n\nAuthor: JIAlonglong\nUpdated: ''' + 
                datetime.now().strftime('%Y-%m-%d'))

        # Actions子图
        with dot.subgraph(name='cluster_actions') as actions:
            actions.attr(label='Actions', style='filled', color='red', fillcolor='#ffebee')
            for name in ['ActionBlock', 'NavigationActionBlock', 'WaitActionBlock', 'MovementActionBlock']:
                actions.node(name, shape='box', style='rounded')

        # UI Components子图
        with dot.subgraph(name='cluster_ui') as ui:
            ui.attr(label='UI Components', style='filled', color='green', fillcolor='#e8f5e9')
            for name in ['MainWindow', 'NavigationPanel', 'MappingPanel', 'ControlPanel', 
                        'SettingsPanel', 'ActionConfigurator', 'SpeedDisplay', 'BatteryIndicator', 
                        'JoystickWidget', 'RVizView']:
                ui.node(name, shape='box', style='rounded')

        # Controllers子图
        with dot.subgraph(name='cluster_controllers') as controllers:
            controllers.attr(label='Controllers', style='filled', color='blue', fillcolor='#e3f2fd')
            for name in ['RobotController', 'NavigationController', 'MappingController']:
                controllers.node(name, shape='box', style='rounded')

        # 添加连接关系
        added_edges = set()
        for class_name, class_info in self.classes.items():
            for connect in class_info.get('connects', []):
                edge_key = (connect['sender'], connect['receiver'], connect['signal'])
                if edge_key not in added_edges:
                    dot.edge(connect['sender'], connect['receiver'], 
                            label=connect['signal'], 
                            style='solid')
                    added_edges.add(edge_key)

        # 保存图形
        try:
            dot.render(f'docs/images/{output_file}', format='png', cleanup=True)
            print("生成信号流图成功!")
        except Exception as e:
            print(f"生成信号流图失败: {str(e)}")

    def update_readme(self, readme_path: str, image_path: str) -> None:
        """更新README文件中的信号流图"""
        with open(readme_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 检查是否已存在信号流图部分
        signal_flow_section = "## 信号流图\n\n![Signal Flow Graph]"
        if signal_flow_section in content:
            # 更新现有图片
            content = re.sub(
                r'!\[Signal Flow Graph\]\([^)]+\)',
                f'![Signal Flow Graph]({image_path})',
                content
            )
        else:
            # 添加新的部分
            content += f"\n\n{signal_flow_section}({image_path})\n"

        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(content)

def main():
    if len(sys.argv) != 2:
        print("用法: python3 signal_flow_analyzer.py <项目根目录>")
        sys.exit(1)

    project_root = sys.argv[1]
    src_dir = os.path.join(project_root, 'src')
    include_dir = os.path.join(project_root, 'include')
    readme_path = os.path.join(project_root, 'README.md')
    output_dir = os.path.join(project_root, 'docs', 'images')
    output_file = os.path.join(output_dir, 'signal_flow')

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    analyzer = SignalFlowAnalyzer()
    print("分析源代码...")
    analyzer.analyze_directory(src_dir)
    analyzer.analyze_directory(include_dir)

    print("生成信号流图...")
    analyzer.generate_graph(output_file)

    print("更新README...")
    analyzer.update_readme(readme_path, 'docs/images/signal_flow.png')

    print("完成!")

if __name__ == '__main__':
    main() 