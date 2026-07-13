#!/bin/bash

# 实例编号，可通过命令行参数指定，默认为 1
INSTANCE_ID="${1:-1}"

if [ -z "$PX4_SRC_DIR" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PX4_SRC_DIR="$(cd "${SCRIPT_DIR}/" && pwd)"
fi

PX4_BUILD_DIR="${PX4_SRC_DIR}/build/px4_sitl_default"

cd "${PX4_BUILD_DIR}/src/modules/simulation/gz_bridge"

export PX4_GZ_MODEL_POSE="$((INSTANCE_ID * 10)),0"
export PX4_SYS_AUTOSTART=4003
export PX4_SIM_MODEL=gz_rc_cessna
export PX4_GZ_WORLD=windy
export GZ_IP=127.0.0.1
export PX4_PARAM_UXRCE_DDS_AG_IP="$(( -1408095744 + INSTANCE_ID ))"

echo "========== 启动 PX4 SITL 单机仿真 =========="
echo "源码目录: ${PX4_SRC_DIR}"
echo "构建目录: ${PX4_BUILD_DIR}"
echo "机体: Cessna"
echo "仿真器: Gazebo"
echo "实例编号: ${INSTANCE_ID}"
echo "=========================================="

"${PX4_BUILD_DIR}/bin/px4" -i "${INSTANCE_ID}"

