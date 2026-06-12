#!/bin/bash

# 检查是否传入了参数
if [ $# -ne 1 ]; then
    echo "用法: $0 <执行次数>"
    echo "示例: $0 5  (表示在5个新窗口中分别执行PX4指令)"
    exit 1
fi

# 获取传入的执行次数参数
EXECUTE_TIMES=$1

# 检查参数是否为正整数
if ! [[ "$EXECUTE_TIMES" =~ ^[1-9][0-9]*$ ]]; then
    echo "错误: 参数必须是一个正整数！"
    exit 1
fi

# 循环创建新窗口并执行命令
for ((i=1; i<=EXECUTE_TIMES; i++))
do
    echo "========== 正在打开第 $i 个终端窗口执行指令 =========="

    # 计算位置偏移
    pose_y=$((i * 10))

    # 构建tmux会话名称
    session_name="px4_$i"

    # 构建完整的 PX4 启动命令
    # 注意：这里使用单引号和双引号的组合来正确处理变量
    px4_command="/bin/sh -c 'cd /home/ubuntu/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge && /usr/bin/cmake -E env PX4_GZ_MODEL_POSE=\"0,$pose_y\" HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 /home/ubuntu/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $i'"

    # 使用tmux后台执行命令
    tmux new-session -d -s "$session_name" "$px4_command"

    echo "已创建会话: $session_name, 位置: (0, $pose_y), 实例ID: $i"

    # 等待一段时间，避免同时启动造成冲突
    sleep 20
done

echo "========== 所有 $EXECUTE_TIMES 个PX4实例已启动 =========="
