#!/bin/bash

# 设置TASKING编译器路径
export PATH="/d/AURIX-Studio-1.10.32/tools/Compilers/Tasking_1.1r8/ctc/bin:$PATH"

echo "=== 开始编译项目 ==="
echo "编译器路径: $PATH"
echo ""

# 进入Debug目录
cd d:/1111111111111/22222/Debug

# 清理项目
echo "1. 清理项目..."
make clean 2>&1

# 编译项目
echo ""
echo "2. 编译项目..."
make all 2>&1

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "=== 编译成功！==="
    echo "输出文件: d:/1111111111111/22222/Debug/Seekfree_TC264_Opensource_Library.elf"
else
    echo ""
    echo "=== 编译失败 ==="
    echo "请检查错误信息"
fi
