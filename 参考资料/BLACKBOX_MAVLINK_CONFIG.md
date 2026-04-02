# Blackbox MAVLink 消息配置指南

## 概述

本文档说明如何配置 PX4 系统以使用自定义的 `blackbox.xml` MAVLink 消息定义文件，该文件包含低带宽 GNSS 定位消息（`GNSS_LOW_BANDWIDTH_POSITION`，ID: 20000）。

## 已完成的配置

✅ 已创建文件：`src/modules/mavlink/mavlink/message_definitions/v1.0/blackbox.xml`

该文件包含：
- 包含 `common.xml` 的所有标准消息
- 新增自定义消息：`GNSS_LOW_BANDWIDTH_POSITION` (ID: 20000)
  - `lat` (uint32_t): 纬度，单位 degE6
  - `lon` (uint32_t): 经度，单位 degE6
  - `alt` (int32_t): 海拔高度，单位米
  - `satellites` (uint8_t): 可见卫星数
  - `fix_type` (uint8_t): GPS 定位类型（使用 GPS_FIX_TYPE 枚举）

## 配置步骤

### 方法 1: 修改板级配置文件（推荐）

根据你的目标硬件，修改对应的 `default.px4board` 文件：

#### SITL 仿真配置

编辑文件：`boards/px4/sitl/default.px4board`

找到或添加以下行：
```bash
CONFIG_MAVLINK_DIALECT="blackbox"
```

#### 硬件板配置

根据你的飞控板型号，编辑对应的配置文件。例如：

- **FMU v6x**: `boards/px4/fmu-v6x/default.px4board`
- **FMU v5x**: `boards/px4/fmu-v5x/default.px4board`
- **Cube Orange**: `boards/cubepilot/cubeorange/default.px4board`
- **Holybro Durandal**: `boards/holybro/durandal-v1/default.px4board`

在文件中添加或修改：
```bash
CONFIG_MAVLINK_DIALECT="blackbox"
```

### 方法 2: 使用 menuconfig 配置

1. 运行配置界面：
```bash
make px4_sitl_default menuconfig
```

2. 导航到：
   ```
   -> MAVLINK_DIALECT
   ```

3. 将值从 `"common"` 修改为 `"blackbox"`

4. 保存并退出

## 编译步骤

配置完成后，清理并重新编译：

### SITL 仿真
```bash
# 清理之前的构建
make px4_sitl_default clean

# 重新编译
make px4_sitl_default
```

### 硬件目标
```bash
# 清理
make <your_board> clean

# 编译
make <your_board>
```

例如：
```bash
make px4_fmu-v6x_default
make px4_fmu-v5x_default
```

## 验证配置

编译成功后，生成的 MAVLink 头文件将位于：
```
build/<target>/mavlink/blackbox/blackbox.h
```

检查该文件中是否包含你的自定义消息定义：
```bash
grep -n "GNSS_LOW_BANDWIDTH_POSITION" build/<target>/mavlink/blackbox/blackbox.h
```

应该能看到类似：
```c
#define MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION 20000
```

## 在代码中使用

### 发送消息示例

```cpp
#include <mavlink/blackbox/blackbox.h>

// 创建消息
mavlink_gnss_low_bandwidth_position_t pos_msg;
pos_msg.lat = latitude * 1000000;  // 转换为 degE6
pos_msg.lon = longitude * 1000000;  // 转换为 degE6
pos_msg.alt = altitude;             // 米
pos_msg.satellites = num_satellites;
pos_msg.fix_type = fix_type;

// 打包消息
uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
uint16_t len = mavlink_msg_gnss_low_bandwidth_position_pack(
    system_id,
    component_id,
    &msg,
    &pos_msg
);

// 发送
mavlink_send_uart(channel, buffer, len);
```

### 接收消息示例

```cpp
#include <mavlink/blackbox/blackbox.h>

mavlink_message_t msg;
mavlink_gnss_low_bandwidth_position_t pos;

// 解析消息
if (mavlink_msg_gnss_low_bandwidth_position_decode(&msg, &pos) == MAVLINK_MSG_ID_GNSS_LOW_BANDWIDTH_POSITION) {
    // 处理接收到的位置数据
    double latitude = pos.lat / 1000000.0;
    double longitude = pos.lon / 1000000.0;
    float altitude = pos.alt;
    uint8_t satellites = pos.satellites;
    uint8_t fix_type = pos.fix_type;
}
```

## 注意事项

1. **消息大小**: `GNSS_LOW_BANDWIDTH_POSITION` 消息仅 16 字节（不含包头和校验），适合低带宽链路

2. **单位转换**:
   - lat/lon: degE6 (乘以 1,000,000)
   - alt: 米 (m)

3. **兼容性**: 地面站软件需要支持自定义消息 ID 20000

4. **同时支持多个 dialect**: 系统会同时生成 `uAvionix` 和你的 `blackbox` dialect

5. **如果修改了 blackbox.xml**: 需要重新编译才能生效

## 故障排查

### 编译错误 "XML file not found"

确保 `blackbox.xml` 文件位于正确位置：
```
src/modules/mavlink/mavlink/message_definitions/v1.0/blackbox.xml
```

### 生成的头文件找不到

清理构建目录并重新编译：
```bash
make px4_sitl_default clean
make px4_sitl_default
```

### 消息 ID 冲突

确保 ID 20000 没有在其他 dialect 中使用。可以搜索：
```bash
grep -r 'id="20000"' src/modules/mavlink/mavlink/message_definitions/
```

## 恢复默认配置

如果需要恢复使用标准的 `common.xml`：

1. 修改 `default.px4board` 文件：
```bash
CONFIG_MAVLINK_DIALECT="common"
```
或者直接删除该行（会使用 Kconfig 中的默认值 "common"）

2. 重新编译：
```bash
make px4_sitl_default clean
make px4_sitl_default
```

## 相关文件

- 消息定义：`src/modules/mavlink/mavlink/message_definitions/v1.0/blackbox.xml`
- CMake 配置：`src/modules/mavlink/CMakeLists.txt`
- Kconfig: `src/modules/mavlink/Kconfig`
- 板级配置：`boards/<vendor>/<board>/default.px4board`

## 参考文档

- [PX4 MAVLink 文档](https://docs.px4.io/main/en/middleware/mavlink.html)
- [MAVLink 消息定义格式](https://mavlink.io/en/guide/xml_schema.html)
