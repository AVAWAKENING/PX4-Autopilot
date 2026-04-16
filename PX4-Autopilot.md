# PX4-Autopilot 项目设计范式

本文档记录PX4飞控系统的设计范式和关键概念，帮助理解项目架构。

---

## EKF2 位置估计有效性判断机制

### 概述

PX4使用EKF2（扩展卡尔曼滤波器）进行位置估计。位置估计的有效性通过"惯性推算超时"机制来判断。当辅助传感器数据丢失时，系统会进入纯惯性推算模式，如果超过设定时间仍无辅助数据，位置估计将被标记为无效。

### 核心概念

#### 1. 本地位置估计 (Local Position)

本地位置是在NED（北东地）坐标系下的位置估计，原点为EKF2启动时的飞行器位置。

**相关消息**: `VehicleLocalPosition.msg`

#### 2. 全局位置估计 (Global Position)

全局位置是在WGS84坐标系下的位置估计（经度、纬度、高度）。

**相关消息**: `VehicleGlobalPosition.msg`

### 有效性判断逻辑

#### 水平位置有效性 (xy_valid)

**判断函数**: `isLocalHorizontalPositionValid()`

**核心代码** ([ekf.h:231-234](src/modules/ekf2/EKF/ekf.h#L231-L234)):
```cpp
bool isLocalHorizontalPositionValid() const
{
    return !_horizontal_deadreckon_time_exceeded;
}
```

**辅助源类型**:

1. **水平位置辅助源** ([estimator_interface.cpp:618-623](src/modules/ekf2/EKF/estimator_interface.cpp#L618-L623)):
   - `gnss_pos`: GPS/GNSS位置
   - `ev_pos`: 视觉位置
   - `aux_gpos`: 辅助全局位置

2. **水平速度辅助源** ([estimator_interface.cpp:625-633](src/modules/ekf2/EKF/estimator_interface.cpp#L625-L633)):
   - `gnss_vel`: GPS/GNSS速度
   - `ev_vel`: 视觉速度
   - `opt_flow`: 光流
   - `fuse_aspd && fuse_beta`: 空速和侧滑角组合（固定翼）

**更新逻辑** ([ekf_helper.cpp:800-898](src/modules/ekf2/EKF/ekf_helper.cpp#L800-L898)):
- 当有辅助源激活且最近有数据融合时，重置超时计时器
- 当所有辅助源丢失超过 `EKF2_NOAID_TOUT` 时间，标记为无效

#### 垂直位置有效性 (z_valid)

**判断函数**: `isLocalVerticalPositionValid()`

**核心代码** ([ekf.h:236-239](src/modules/ekf2/EKF/ekf.h#L236-L239)):
```cpp
bool isLocalVerticalPositionValid() const
{
    return !_vertical_position_deadreckon_time_exceeded;
}
```

**辅助源类型** ([estimator_interface.cpp:656-662](src/modules/ekf2/EKF/estimator_interface.cpp#L656-L662)):
- `gnss_hgt`: GPS高度
- `baro_hgt`: 气压高度
- `rng_hgt`: 测距仪高度
- `ev_hgt`: 视觉高度

**更新逻辑** ([ekf_helper.cpp:900-919](src/modules/ekf2/EKF/ekf_helper.cpp#L900-L919)):
- 当有垂直位置辅助源激活时，重置超时计时器
- 当所有垂直位置辅助源丢失超过 `EKF2_NOAID_TOUT` 时间，标记为无效

#### 垂直速度有效性 (v_z_valid)

**判断函数**: `isLocalVerticalVelocityValid()`

**核心代码** ([ekf.h:241-244](src/modules/ekf2/EKF/ekf.h#L241-L244)):
```cpp
bool isLocalVerticalVelocityValid() const
{
    return !_vertical_velocity_deadreckon_time_exceeded;
}
```

**辅助源类型** ([estimator_interface.cpp:674-678](src/modules/ekf2/EKF/estimator_interface.cpp#L674-L678)):
- `gnss_vel`: GPS/GNSS速度
- `ev_vel`: 视觉速度

**更新逻辑**:
- 当有垂直速度辅助源激活时，重置超时计时器
- 当所有垂直速度辅助源丢失超过超时时间，**且**垂直位置已无效时，标记为无效

#### 全局水平位置有效性 (lat_lon_valid)

**判断函数**: `isGlobalHorizontalPositionValid()`

**核心代码** ([ekf.h:221-224](src/modules/ekf2/EKF/ekf.h#L221-L224)):
```cpp
bool isGlobalHorizontalPositionValid() const
{
    return _local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid();
}
```

**条件**:
1. 本地原点经纬度已初始化
2. 本地水平位置有效

#### 全局垂直位置有效性 (alt_valid)

**判断函数**: `isGlobalVerticalPositionValid()`

**核心代码** ([ekf.h:226-229](src/modules/ekf2/EKF/ekf.h#L226-L229)):
```cpp
bool isGlobalVerticalPositionValid() const
{
    return PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid();
}
```

**条件**:
1. 本地原点高度有效（非NaN/Inf）
2. 本地垂直位置有效

### 关键参数

#### EKF2_NOAID_TOUT

**定义位置**: [module.yaml:76-86](src/modules/ekf2/module.yaml#L76-L86)

**参数说明**:
- **描述**: 最大惯性推算时间
- **默认值**: 5,000,000 微秒（5秒）
- **范围**: 500,000 - 10,000,000 微秒（0.5-10秒）
- **单位**: 微秒
- **作用**: 当辅助传感器数据丢失后，系统进入纯惯性推算模式。如果超过此时间仍无辅助数据，位置估计将被标记为无效。

**使用位置** ([common.h:482](src/modules/ekf2/EKF/common.h#L482)):
```cpp
int32_t valid_timeout_max{5'000'000};  // 默认5秒
```

### 消息发布逻辑

位置估计有效性标志在EKF2模块中设置并发布：

**VehicleLocalPosition** ([EKF2.cpp:1566-1571](src/modules/ekf2/EKF2.cpp#L1566-L1571)):
```cpp
lpos.xy_valid = _ekf.isLocalHorizontalPositionValid();
lpos.v_xy_valid = _ekf.isLocalHorizontalPositionValid();
lpos.z_valid = _ekf.isLocalVerticalPositionValid() || _ekf.isLocalVerticalVelocityValid();
lpos.v_z_valid = _ekf.isLocalVerticalVelocityValid() || _ekf.isLocalVerticalPositionValid();
```

**VehicleGlobalPosition** ([EKF2.cpp:1179-1182](src/modules/ekf2/EKF2.cpp#L1179-L1182)):
```cpp
global_pos.lat_lon_valid = _ekf.isGlobalHorizontalPositionValid();
global_pos.alt_valid = _ekf.isGlobalVerticalPositionValid();
```

### 设计要点

1. **分层判断**: 本地位置有效性是基础，全局位置有效性依赖于本地位置有效性
2. **超时机制**: 使用时间超时来判断辅助源是否有效，避免硬性切换
3. **多源融合**: 支持多种辅助源，提高系统鲁棒性
4. **速度与位置分离**: 垂直方向的速度和位置有效性分开判断，提供更灵活的状态管理

### 相关文件

- **核心逻辑**: 
  - `src/modules/ekf2/EKF/ekf.h` - 有效性判断函数定义
  - `src/modules/ekf2/EKF/ekf_helper.cpp` - 死推算状态更新
  - `src/modules/ekf2/EKF/estimator_interface.cpp` - 辅助源计数
  
- **消息定义**:
  - `msg/versioned/VehicleLocalPosition.msg` - 本地位置消息
  - `msg/versioned/VehicleGlobalPosition.msg` - 全局位置消息
  
- **参数配置**:
  - `src/modules/ekf2/module.yaml` - EKF2参数定义
  - `src/modules/ekf2/EKF/common.h` - 参数默认值

---

*最后更新: 2026-04-13*
