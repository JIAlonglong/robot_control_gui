#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 检查Python3是否安装
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3, 请先安装python3"
    exit 1
fi

# 设置执行权限
chmod +x "$SCRIPT_DIR/add_license.py"

# 运行Python脚本
echo "添加MIT许可证头到源文件..."
python3 "$SCRIPT_DIR/add_license.py" "$PROJECT_ROOT/src" "$PROJECT_ROOT/include"

echo "完成!" 