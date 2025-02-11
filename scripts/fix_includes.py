#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys

def get_include_path(file_path, header_file):
    """根据源文件路径和头文件名确定正确的包含路径"""
    # 获取项目根目录
    project_root = os.path.dirname(os.path.dirname(file_path))
    include_dir = os.path.join(project_root, 'include')
    
    # 在include目录下搜索头文件
    for root, dirs, files in os.walk(include_dir):
        if header_file in files:
            # 获取相对于include目录的路径
            rel_path = os.path.relpath(os.path.join(root, header_file), include_dir)
            return rel_path
    return None

def fix_includes(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 查找所有的include语句
    include_pattern = r'#include\s+"([^"]+)"'
    includes = re.findall(include_pattern, content)
    
    new_content = content
    modified = False
    
    for include in includes:
        # 获取头文件名
        header_file = os.path.basename(include)
        # 获取正确的包含路径
        correct_path = get_include_path(file_path, header_file)
        
        if correct_path:
            # 替换原有的包含语句
            old_include = f'#include "{include}"'
            new_include = f'#include "{correct_path}"'
            if old_include != new_include:
                new_content = new_content.replace(old_include, new_include)
                modified = True
                print(f"In {file_path}: {old_include} -> {new_include}")
    
    if modified:
        print(f"Fixing includes in {file_path}")
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(new_content)

def process_directory(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.h')):
                file_path = os.path.join(root, file)
                fix_includes(file_path)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 fix_includes.py <directory>")
        sys.exit(1)
    
    directory = sys.argv[1]
    if not os.path.isdir(directory):
        print(f"Error: {directory} is not a directory")
        sys.exit(1)
    
    process_directory(directory)
    print("Done fixing includes") 