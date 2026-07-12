#!/bin/bash

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

echo "========== 启动 PX4 SITL 单机仿真 =========="
echo "源码目录: ${PX4_SRC_DIR}"
echo "构建目录: ${PX4_BUILD_DIR}"
echo "机体: Cessna"
echo "仿真器: Gazebo"
echo "=========================================="

"${PX4_BUILD_DIR}/bin/px4" -i 1

