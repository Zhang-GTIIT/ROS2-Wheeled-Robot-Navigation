#!/bin/bash

# 检查是否提供了参数
if [ $# -eq 0 ]; then
    echo "Usage: $0 <python_file.py> [args...]"
    echo "Example: $0 script.py --input data.csv"
    exit 1
fi

# 获取第一个参数作为 Python 文件路径
PYTHON_FILE="$1"
shift  # 移除第一个参数，剩余的作为传递给 Python 脚本的参数

# 检查文件是否存在
if [ ! -f "$PYTHON_FILE" ]; then
    echo "Error: File '$PYTHON_FILE' does not exist."
    exit 2
fi

# 检查文件是否以 .py 结尾（可选）
if [[ "$PYTHONFILE" != *.py ]]; then
    echo "Warning: '$PYTHON_FILE' does not appear to be a Python file (missing .py extension)."
    # 可选择继续或退出，这里选择继续执行
fi

# 使用 python3 运行脚本（推荐使用 python3）
python3 "$PYTHON_FILE" "$@"