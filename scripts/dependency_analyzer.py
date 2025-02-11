#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
依赖关系分析工具

此工具用于分析代码库的依赖关系,支持多种编程语言。
目前主要支持:
- C++文件 (.cpp, .h)
- Python文件 (.py)
- CMake文件 (CMakeLists.txt)

使用方法:
python3 dependency_analyzer.py [options] <project_path>

选项:
--lang: 指定要分析的语言 (cpp, python, all)
--depth: 依赖分析的深度级别 (1-直接依赖, 2-间接依赖, 3-完整依赖树)
--output: 输出格式 (tree, list, dot)
"""

import os
import re
import sys
import argparse
from collections import defaultdict
from typing import Dict, List, Set, Tuple

class DependencyAnalyzer:
    def __init__(self):
        self.dependencies = defaultdict(set)
        self.reverse_dependencies = defaultdict(set)
        self.file_types = {
            'cpp': ['.cpp', '.h', '.hpp'],
            'python': ['.py'],
            'cmake': ['CMakeLists.txt']
        }
        
    def analyze_file(self, file_path: str) -> Set[str]:
        """分析单个文件的依赖关系"""
        ext = os.path.splitext(file_path)[1]
        deps = set()
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
            if ext in self.file_types['cpp']:
                deps.update(self._analyze_cpp(content))
            elif ext in self.file_types['python']:
                deps.update(self._analyze_python(content))
            elif file_path.endswith('CMakeLists.txt'):
                deps.update(self._analyze_cmake(content))
                
        except Exception as e:
            print(f"分析文件 {file_path} 时出错: {str(e)}")
            
        return deps
        
    def _analyze_cpp(self, content: str) -> Set[str]:
        """分析C++文件的依赖"""
        deps = set()
        
        # 匹配 #include 语句
        include_pattern = r'#include\s*[<"]([^>"]+)[>"]'
        matches = re.finditer(include_pattern, content)
        
        for match in matches:
            header = match.group(1)
            deps.add(header)
            
        return deps
        
    def _analyze_python(self, content: str) -> Set[str]:
        """分析Python文件的依赖"""
        deps = set()
        
        # 匹配 import 语句
        import_pattern = r'^(?:from\s+(\S+)\s+import|\s*import\s+([^#\n]+))'
        matches = re.finditer(import_pattern, content, re.MULTILINE)
        
        for match in matches:
            module = match.group(1) or match.group(2)
            deps.add(module.strip())
            
        return deps
        
    def _analyze_cmake(self, content: str) -> Set[str]:
        """分析CMake文件的依赖"""
        deps = set()
        
        # 匹配 find_package 语句
        package_pattern = r'find_package\s*\(\s*(\w+)'
        matches = re.finditer(package_pattern, content)
        
        for match in matches:
            package = match.group(1)
            deps.add(package)
            
        return deps
        
    def analyze_project(self, project_path: str, file_types: List[str] = None) -> None:
        """分析整个项目的依赖关系"""
        if file_types is None:
            file_types = [ext for types in self.file_types.values() for ext in types]
            
        for root, _, files in os.walk(project_path):
            for file in files:
                if any(file.endswith(ext) for ext in file_types):
                    file_path = os.path.join(root, file)
                    rel_path = os.path.relpath(file_path, project_path)
                    deps = self.analyze_file(file_path)
                    
                    if deps:
                        self.dependencies[rel_path] = deps
                        for dep in deps:
                            self.reverse_dependencies[dep].add(rel_path)
                            
    def print_dependencies(self, depth: int = 1, output_format: str = 'tree') -> None:
        """打印依赖关系"""
        if output_format == 'tree':
            self._print_tree(depth)
        elif output_format == 'list':
            self._print_list()
        elif output_format == 'dot':
            self._print_dot()
            
    def _print_tree(self, depth: int, file: str = None, level: int = 0, visited: Set[str] = None) -> None:
        """以树状结构打印依赖"""
        if visited is None:
            visited = set()
            
        if file is None:
            # 打印所有根节点
            for f in sorted(self.dependencies.keys()):
                if f not in self.reverse_dependencies:
                    self._print_tree(depth, f, 0, visited.copy())
            return
            
        if file in visited or level >= depth:
            return
            
        visited.add(file)
        prefix = '  ' * level + ('├── ' if level > 0 else '')
        print(f"{prefix}{file}")
        
        if file in self.dependencies:
            deps = sorted(self.dependencies[file])
            for dep in deps:
                self._print_tree(depth, dep, level + 1, visited.copy())
                
    def _print_list(self) -> None:
        """以列表形式打印依赖"""
        for file, deps in sorted(self.dependencies.items()):
            print(f"\n{file}:")
            for dep in sorted(deps):
                print(f"  - {dep}")
                
    def _print_dot(self) -> None:
        """以DOT格式打印依赖关系图"""
        print("digraph DependencyGraph {")
        print("  node [shape=box];")
        
        # 添加节点
        for file in self.dependencies.keys():
            print(f'  "{file}";')
            
        # 添加边
        for file, deps in self.dependencies.items():
            for dep in deps:
                print(f'  "{file}" -> "{dep}";')
                
        print("}")

def main():
    parser = argparse.ArgumentParser(description='代码依赖关系分析工具')
    parser.add_argument('project_path', help='项目根目录路径')
    parser.add_argument('--lang', choices=['cpp', 'python', 'all'], default='all',
                      help='要分析的编程语言')
    parser.add_argument('--depth', type=int, default=1,
                      help='依赖分析的深度级别 (1-直接依赖, 2-间接依赖, 3-完整依赖树)')
    parser.add_argument('--output', choices=['tree', 'list', 'dot'], default='tree',
                      help='输出格式')
    
    args = parser.parse_args()
    
    # 确定要分析的文件类型
    file_types = []
    if args.lang == 'cpp':
        file_types.extend(['.cpp', '.h', '.hpp'])
    elif args.lang == 'python':
        file_types.extend(['.py'])
    elif args.lang == 'all':
        file_types = None
    
    analyzer = DependencyAnalyzer()
    print(f"\n正在分析项目: {args.project_path}")
    analyzer.analyze_project(args.project_path, file_types)
    print("\n依赖关系分析结果:")
    analyzer.print_dependencies(args.depth, args.output)

if __name__ == '__main__':
    main() 