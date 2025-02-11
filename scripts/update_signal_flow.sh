#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 检查Python3是否安装
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3, 请先安装python3"
    exit 1
fi

# 检查graphviz是否安装
if ! command -v dot &> /dev/null; then
    echo "错误: 未找到graphviz, 请先安装: sudo apt-get install graphviz"
    exit 1
fi

# 检查python依赖
python3 -c "import graphviz" 2>/dev/null || {
    echo "正在安装python-graphviz..."
    pip3 install graphviz
}

# 设置执行权限
chmod +x "$SCRIPT_DIR/signal_flow_analyzer.py"

# 运行分析工具
echo "开始分析信号流..."
python3 "$SCRIPT_DIR/signal_flow_analyzer.py" "$PROJECT_ROOT"

# 如果生成成功,添加到git
if [ $? -eq 0 ]; then
    echo "信号流图已更新"
    if command -v git &> /dev/null && [ -d "$PROJECT_ROOT/.git" ]; then
        cd "$PROJECT_ROOT"
        git add docs/images/signal_flow.png README.md
        git commit -m "更新信号流图" || true
    fi
fi 