#!/bin/bash

# 格式化所有C++源文件和头文件
find src include -type f \( -name "*.cpp" -o -name "*.h" \) -exec clang-format -i -style=file {} \;

echo "代码格式化完成!" 