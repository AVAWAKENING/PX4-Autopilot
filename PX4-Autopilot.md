# PX4 事件系统禁用指南

## 概述

本文档说明如何在 PX4 中完全禁用事件（不通过 MAVLink 发送，也不记录到日志）。

## 事件日志级别

PX4 事件系统使用两个独立的日志级别：

1. **External Level** - 控制是否通过 MAVLink 发送到地面站
2. **Internal Level** - 控制是否记录到飞行日志

## 完全禁用事件的方法

### 方法 1：使用 `events::Log::Disabled`（推荐）

最简单的方式是只传入 `events::Log::Disabled`，这样 external 和 internal 都会被设置为 Disabled：

```cpp
events::send(events::ID("event_name"), 
             events::Log::Disabled, 
             "消息内容");
```

**原理**：`LogLevels` 的单参数构造函数会将 internal 设置为与 external 相同的值。

### 方法 2：显式设置两个级别都为 Disabled

```cpp
events::send(events::ID("event_name"), 
             {events::Log::Disabled, events::LogInternal::Disabled}, 
             "消息内容");
```

这种方式更明确，代码可读性更好。

## 实际示例

### 示例 1：完全禁用调试事件

**修改前**：
```cpp
events::send(events::ID("module_debug_info"), 
             events::Log::Debug, 
             "调试信息");
```

**修改后**：
```cpp
events::send(events::ID("module_debug_info"), 
             events::Log::Disabled, 
             "调试信息 - 已禁用");
```

### 示例 2：禁用警告事件

**修改前**：
```cpp
events::send(events::ID("mavlink_mission_storage_write_failure2"), 
             events::Log::Critical,
             "Storage write failure");
```

**修改后**：
```cpp
events::send(events::ID("mavlink_mission_storage_write_failure2"), 
             events::Log::Disabled,
             "Storage write failure - 已禁用");
```

### 示例 3：根据条件动态禁用

```cpp
// 根据配置决定是否启用
bool enable_events = false; // 从参数或配置读取

auto log_level = enable_events ? 
                 events::Log::Info : 
                 events::Log::Disabled;

events::send(events::ID("module_conditional_event"), 
             log_level, 
             "条件事件");
```

## 禁用效果验证

### 1. MAVLink 不发送

在 `src/modules/mavlink/mavlink_main.cpp` 中，external 为 Disabled 的事件会被过滤：

```cpp
if (events::externalLogLevel(orb_event.log_levels) == events::LogLevel::Disabled) {
    continue; // 不插入到 event buffer，不发送
}
```

### 2. 日志不记录

在 `src/modules/logger/logger.cpp` 中，internal 为 Disabled 的事件会被跳过：

```cpp
if (events::internalLogLevel(orb_event->log_levels) == events::LogLevelInternal::Disabled) {
    ++_event_sequence_offset; // skip this event
    // 不写入日志文件
}
```

## 常见禁用场景

### 场景 1：调试期间的事件

```cpp
// 调试完成后禁用
events::send(events::ID("module_init_debug"), 
             events::Log::Disabled, 
             "模块初始化调试信息");
```

### 场景 2：频繁触发的内部事件

```cpp
// 频繁触发的事件，避免刷屏
events::send(events::ID("sensor_data_received"), 
             events::Log::Disabled, 
             "传感器数据接收");
```

### 场景 3：已废弃但保留的事件

```cpp
// 保留事件定义但不激活
events::send(events::ID("legacy_warning"), 
             {events::Log::Disabled, events::LogInternal::Disabled}, 
             "已废弃的警告");
```

## 日志级别对照表

| 级别 | External (MAVLink) | Internal (日志) | 说明 |
|------|-------------------|----------------|------|
| `events::Log::Disabled` | ❌ 不发送 | ❌ 不记录 | 完全禁用 |
| `{Log::Disabled, LogInternal::Info}` | ❌ 不发送 | ✅ 记录为 Info | 仅日志 |
| `{Log::Info, LogInternal::Disabled}` | ✅ 发送 | ❌ 不记录 | 仅通知 |
| `events::Log::Info` | ✅ 发送 | ✅ 记录为 Info | 默认行为 |

## 注意事项

1. **uORB 仍会发布**：即使设置为 Disabled，事件仍会发布到 uORB topic，只是在 MAVLink 和 logger 中被过滤
2. **性能影响**：如果完全不需要某个事件，最好直接删除 `events::send()` 调用
3. **事件 ID 冲突**：禁用的事件 ID 仍然被占用，避免后续使用相同名称
4. **向后兼容**：保留禁用的事件定义有助于代码维护和未来启用

## 批量禁用示例

如果需要批量禁用某个模块的所有事件：

```cpp
// 在模块头文件定义日志级别
#ifdef DISABLE_MODULE_EVENTS
    #define MODULE_LOG_LEVEL events::Log::Disabled
#else
    #define MODULE_LOG_LEVEL events::Log::Info
#endif

// 使用时
events::send(events::ID("module_event1"), 
             MODULE_LOG_LEVEL, 
             "事件 1");

events::send(events::ID("module_event2"), 
             MODULE_LOG_LEVEL, 
             "事件 2");
```

## 相关文件

- 事件定义头文件：`platforms/common/include/px4_platform_common/events.h`
- 生成的事件定义：`src/lib/events/libevents/libs/cpp/generated/events_generated.h`
- MAVLink 事件处理：`src/modules/mavlink/mavlink_events.cpp`
- 日志事件处理：`src/modules/logger/logger.cpp`

## 总结

完全禁用事件的标准做法：

```cpp
events::send(events::ID("event_name"), 
             events::Log::Disabled, 
             "消息内容");
```

这样可以确保事件既不发送到地面站，也不记录到飞行日志，实现完全禁用。
