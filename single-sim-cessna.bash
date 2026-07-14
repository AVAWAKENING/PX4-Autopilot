#!/bin/bash

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "启动 PX4 SITL 单机仿真 (Cessna)"
    echo ""
    echo "选项:"
    echo "  -u, --uxrce_ip IP    设置 UXRCE_DDS_AG_IP 参数 (IPv4 格式，默认: 172.18.21.1)"
    echo "  -h, --help           显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                           # 使用默认 IP 172.18.21.1"
    echo "  $0 -u 192.168.1.100          # 自定义 DDS IP"
    echo "  $0 --uxrce_ip 10.0.0.5       # 使用完整参数名"
}

# IP 地址转 int32 (有符号)
# 参考: Tools/convert_ip.py
ip_to_int32() {
    local ip="$1"
    local IFS='.'
    read -ra parts <<< "$ip"

    if [ ${#parts[@]} -ne 4 ]; then
        echo "错误: 无效的 IP 地址格式" >&2
        exit 1
    fi

    # 转换为整数 (big-endian)
    local int32=0
    for part in "${parts[@]}"; do
        if ! [[ "$part" =~ ^[0-9]+$ ]] || [ "$part" -lt 0 ] || [ "$part" -gt 255 ]; then
            echo "错误: 无效的 IP 地址格式" >&2
            exit 1
        fi
        int32=$(( (int32 << 8) | part ))
    done

    # 处理负数 (最高位为1时)
    if [ $((int32 & 0x80000000)) -ne 0 ]; then
        int32=$(( -0x100000000 + int32 ))
    fi

    echo "$int32"
}

# 默认 IP
DEFAULT_IP="172.18.21.1"
UXRCE_IP=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        -u|--uxrce_ip)
            if [ -z "$2" ]; then
                echo "错误: $1 需要一个 IP 地址参数" >&2
                exit 1
            fi
            UXRCE_IP="$2"
            shift 2
            ;;
        -*)
            echo "错误: 未知参数 $1" >&2
            show_help
            exit 1
            ;;
        *)
            echo "错误: 未知参数 $1" >&2
            show_help
            exit 1
            ;;
    esac
done

if [ -z "$PX4_SRC_DIR" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PX4_SRC_DIR="$(cd "${SCRIPT_DIR}/" && pwd)"
fi

PX4_BUILD_DIR="${PX4_SRC_DIR}/build/px4_sitl_default"

cd "${PX4_BUILD_DIR}/src/modules/simulation/gz_bridge"

export PX4_GZ_MODEL_POSE="0,0"
export PX4_SYS_AUTOSTART=4003
export PX4_SIM_MODEL=gz_rc_cessna
export PX4_GZ_WORLD=windy
export GZ_IP=127.0.0.1

# 设置 UXRCE_DDS_AG_IP
if [ -n "$UXRCE_IP" ]; then
    export PX4_PARAM_UXRCE_DDS_AG_IP="$(ip_to_int32 "$UXRCE_IP")"
else
    export PX4_PARAM_UXRCE_DDS_AG_IP="$(ip_to_int32 "$DEFAULT_IP")"
    UXRCE_IP="$DEFAULT_IP"
fi

echo "========== 启动 PX4 SITL 单机仿真 =========="
echo "源码目录: ${PX4_SRC_DIR}"
echo "构建目录: ${PX4_BUILD_DIR}"
echo "机体: Cessna"
echo "仿真器: Gazebo"
echo "DDS IP: ${UXRCE_IP} (${PX4_PARAM_UXRCE_DDS_AG_IP})"
echo "=========================================="

"${PX4_BUILD_DIR}/bin/px4"
