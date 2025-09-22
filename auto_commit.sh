#!/bin/bash

# 国际象棋机器人项目自动提交脚本
# Auto commit script for AI Chess Robot project

function auto_commit() {
    local 提交信息="$1"

    if [ -z "$提交信息" ]; then
        echo "❌ 错误: 请提供提交信息"
        echo "用法: auto_commit \"你的提交信息\""
        return 1
    fi

    echo "🔄 开始自动提交流程..."
    echo "📝 提交信息: $提交信息"

    # 添加所有文件到暂存区
    echo "📁 添加文件到暂存区..."
    git add .

    # 检查是否有文件需要提交
    if git diff --cached --quiet; then
        echo "⚠️  没有检测到文件更改，跳过提交"
        return 0
    fi

    # 显示将要提交的文件
    echo "📋 将要提交的文件:"
    git diff --cached --name-only | sed 's/^/  ✓ /'

    # 提交更改
    echo "💾 提交更改..."
    git commit -m "$提交信息"

    if [ $? -eq 0 ]; then
        echo "✅ 本地提交成功"

        # 推送到远程仓库
        echo "🚀 推送到远程仓库..."
        git push origin main

        if [ $? -eq 0 ]; then
            echo "🎉 推送成功! 更改已保存到 GitHub"
            echo "🔗 仓库地址: https://github.com/huahua9185/aichess_bot.git"
        else
            echo "⚠️  推送失败，但本地提交已完成"
            echo "💡 可能需要先执行 git pull 或检查网络连接"
        fi
    else
        echo "❌ 提交失败"
        return 1
    fi
}

# 如果脚本直接运行而不是被source，则执行提交
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    if [ $# -eq 0 ]; then
        echo "用法: $0 \"提交信息\""
        echo "例如: $0 \"功能: 完成chess_engine节点开发\""
        exit 1
    fi
    auto_commit "$1"
fi

echo "🛠️  自动提交脚本已加载"
echo "💡 使用方法: auto_commit \"你的提交信息\""