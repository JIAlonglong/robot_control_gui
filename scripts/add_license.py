#!/usr/bin/env python3

import os
import re
from datetime import datetime

MIT_LICENSE = """/*
 * Copyright (c) 2025 JIAlonglong
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
 */

"""

def add_license_to_file(file_path):
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 检查是否已经有版权声明
    if "Copyright (c)" in content:
        print(f"Skip {file_path}: already has copyright notice")
        return
    
    # 添加许可证头部
    new_content = MIT_LICENSE + content
    
    # 写回文件
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"Added license to {file_path}")

def process_directory(directory):
    # 遍历目录下的所有文件
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.h')):
                file_path = os.path.join(root, file)
                add_license_to_file(file_path)

def main():
    # 项目根目录
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # 处理include目录
    include_dir = os.path.join(project_root, 'include')
    if os.path.exists(include_dir):
        process_directory(include_dir)
    
    # 处理src目录
    src_dir = os.path.join(project_root, 'src')
    if os.path.exists(src_dir):
        process_directory(src_dir)
    
    print("License addition completed!")

if __name__ == "__main__":
    main() 