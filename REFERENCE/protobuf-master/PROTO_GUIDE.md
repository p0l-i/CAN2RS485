# Protobuf 协议维护指南

本项目的通信核心定义在 `D:\Protobuf\server_config\protos` 目录下的 `.proto` 文件中。
为了保证服务器端（Telegraf -> InfluxDB）的数据解析正常，**请严格遵守以下规范进行修改**。

## 1. 核心文件

*   `fsae_telemetry.proto`: 定义单 Topic 使用的顶层 `TelemetryFrame` 及其嵌套消息。
*   `fsae_telemetry.options`: 定义 Nanopb（STM32使用）的特定选项，如数组最大长度。

## 2. 修改规范 (CRITICAL)

服务器端的 Telegraf 配置文件 (`telegraf.conf`) 使用了 XPath 来提取 Protobuf 数据。这意味着：

1.  **禁止修改仍在服务器 XPath 中使用的字段名称和类型**：
    *   例如：`uint32 module_id = 1;` 中的 `module_id` 被写入在 `telegraf.conf` 中。如果你把它改名为 `id`，服务器将无法解析该字段，数据会丢失。
    *   如果必须修改，你**必须**同步修改服务器上 `telegraf.conf` 中的 `xpath` 映射，并重启 Telegraf 容器。

2.  **禁止复用已经删除的字段 ID**：
    *   Protobuf 依赖 ID (`= 1`, `= 2`) 来序列化。修改 ID 会导致新旧版本不兼容。
    *   当前 `TelemetryFrame` 中原始 `1~14` 号位已删除并改为 `reserved`，这些编号不能再拿来放新字段。

3.  **新增字段**：
    *   保持单 Topic 时，优先继续在 `TelemetryFrame` 末尾追加新的嵌套消息字段，使用新的 ID。
    *   例如：`VehicleState vehicle_state = 28;`
    *   **注意**：新增字段后，如果能在服务器数据库看到它，还需要手动修改服务器的 `telegraf.conf`，添加对应的 XML Path 映射。否则服务器只会忽略这个新数据，虽然不会报错。
    *   **单 Topic**方案：`TelemetryFrame` 既承载基础遥测，也承载 `modules` 中的 BMS 详细数据。新增字段时同时检查 `telemetry` 和 `bms_data` 两个 measurement 的采集逻辑。
    *   带宽有限，优先保留上云必须的摘要量。当前 BMS 摘要字段只保留：`battery_soc`、最大/最小单体电压及编号、最大/最小温度及编号、`battery_fault_code`。
    *   当前约定：BMS 相关字段继续沿用老结构，`modules = 15` 和 `16~25` 的摘要字段不要随意改成新的 repeated 形状。

4.  **数组长度控制**：
    *   如果有 `repeated` 字段，必须在 `fsae_telemetry.options` 中指定 `max_count`。这是为了让 STM32 (C语言) 能够静态分配内存。

## 3. 现有结构参考

**fsae_telemetry.proto:**

```protobuf
syntax = "proto3";
package fsae;

message BatteryModule {
    uint32 module_id = 1;
    // 23 节电芯电压
    uint32 v01 = 2;
    ...
    // 8 个温度
    sint32 t1 = 30;
    ...
}

message TelemetryFrame {
    reserved 1 to 14;

    // BMS 详细数据
    repeated BatteryModule modules = 15;

    // BMS 摘要
    uint32 battery_soc = 16;
    ...

    // v2 追加字段
    PacketHeader header = 26;
    FastTelemetry fast_telemetry = 27;
    VehicleState vehicle_state = 28;
    ThermalSummary thermal_summary = 29;
    repeated Alarm alarms = 30;
}
```

**fsae_telemetry.options (Nanopb):**

```plaintext
fsae.Alarm.message                 max_size:64
fsae.TelemetryFrame.modules        max_count:6
fsae.TelemetryFrame.alarms         max_count:8
fsae.ThermalSensorSummary.chunks   max_count:4
fsae.ThermalSummary.sensors        max_count:4
fsae.VehicleState.motors           max_count:4
```

## 4. 常见操作流程

### 场景：我想加一个“整车状态”嵌套数据

1.  **修改 Proto**: 在 `fsae_telemetry.proto` 的 `TelemetryFrame` 末尾添加新的 message 字段：
    ```protobuf
    VehicleState vehicle_state = 28;
    ```
2.  **生成代码**:
    *   **Python (本地模拟)**: 运行 `protoc --python_out=. fsae_telemetry.proto`，这会生成` fsae_telemetry_pb2.py`
    *   **STM32 (车载)**: 运行 Nanopb 生成器，更新 STM32 工程中的 `.c/.h` 文件。
3.  **修改服务器配置 (重要)**:
    *   登录服务器，编辑 `server_config/telegraf/telegraf.conf`。
    *   在 `[[inputs.mqtt_consumer.xpath.fields]]` 下添加：
        ```toml
        speed_kmh = "number(//vehicle_state/speed_kmh)"
        ```
    *   重启 Telegraf: `docker-compose restart telegraf`
