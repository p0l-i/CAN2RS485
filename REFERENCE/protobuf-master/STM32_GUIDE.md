# STM32 车载发送端开发指南

车端使用 **Nanopb** 对 `TelemetryFrame` 进行编码。

## 1. 文件位置

已将需要的所有文件整理在 `stm32_code` 目录下。

*   `fsae_telemetry.proto`: 数据定义源文件
*   `fsae_telemetry.options`: Nanopb 配置文件
*   `fsae_telemetry.pb.c` / `.h`: C 代码（更改数据定义`.proto`或`.options`后需要重新生成）
*   `nanopb_lib/`:包含了 Nanopb 的核心库文件 (`pb_encode.c`, `pb_common.c` 等)

## 2. 如何生成 STM32 代码 (如果需要更新)

准备环境： [Nanopb](https://jpa.kapsi.fi/nanopb/)，将生成器目录加入 PATH，或者直接调用 `nanopb_generator.py`。

如果你修改了 `.proto` 文件，你需要使用 Nanopb 提供的生成器脚本重新生成 C 代码。

### 生成命令

```bash
# 假设你已经安装了 nanopb_generator
nanopb_generator fsae_telemetry.proto
```

或者使用 Python 运行（如果你有 Python 环境和 protobuf 库）：

```bash
python path/to/nanopb/generator/nanopb_generator.py fsae_telemetry.proto
```

当前机器目录（macOS）示例：

```bash
python /Users/poli/nanopb-0.4.9.1-macosx-x86/generator/nanopb_generator.py fsae_telemetry.proto
```
### 产物

执行成功后，你会得到：
*   `fsae_telemetry.pb.c`
*   `fsae_telemetry.pb.h`

当前 `CAN2RS485` 工程已将生成后的文件放在 `Middlewares/Third_Party/nanopb/` 下参与构建；如果重新生成，请同步替换工程中的对应文件。

## 3. 工程依赖

除了上述生成的两个文件，你的 STM32 工程还需要 Nanopb 的核心库文件（这些文件一般不需要更新，除非升级 Nanopb 版本）：
*   `pb.h`
*   `pb_common.c`
*   `pb_common.h`
*   `pb_encode.c`
*   `pb_encode.h`
*   `pb_decode.c` (如果车上也需要接收指令，则需要这个；如果只发送，可以不需要)
*   `pb_decode.h`

当前工程只发送遥测，已接入：
*   `pb_common.c/.h`
*   `pb_encode.c/.h`
*   `pb.h`
*   `fsae_telemetry.pb.c/.h`

## 4. 约束

- `repeated` 字段必须在 `.options` 里配置 `max_count`
- `frame.modules_count` 必须显式设置，且不能超过上限
- 不要手改生成的 `.pb.c/.h`
- 当前链路为单 Topic：`fsae/telemetry`
- 非必要不要新增字符串和布尔字段

## 5. 当前 CAN2RS485 工程

现状如下：

- 输出：`USART2` 半双工 RS485 到 DTU
- 发送对象：统一 `TelemetryFrame`
- 发送策略：基础帧 10Hz，带 `modules` 的帧 2Hz
- 已实现输入：
  - 旧主控 `CAN1` 电压、温度、总览、极值、状态、告警
  - 霍尔电流 `0x03C0`
  - `CAN2` 的 `0x18FF50E5`、`0x401`、`0x402`
- 默认优先兼容旧主控；检测到新主控专用帧后切换协议分支
- 485 时序保持 `DIR 高 -> UART 发送 -> 等待 TC -> DIR 低`

### 5.1 当前限制

- `CAN2` 只做被动监听
- `0x18A*`、`0x188350F5` 还没有完整闭环和应答帧
- `.proto` 还没有覆盖新主控全部诊断细节

## 6. 示例

```c
#include "pb_encode.h"
#include "fsae_telemetry.pb.h"

void send_telemetry() {
    uint8_t buffer[512];
    TelemetryFrame message = TelemetryFrame_init_zero;

    // 1. 填充数据
    message.timestamp_ms = HAL_GetTick();
    message.frame_id = frame_counter++;
    message.apps_position = get_apps_pedal(); // float
    message.motor_rpm = get_motor_rpm();      // int32
    
    // ... 填充其他 ...

    // 2. 序列化
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, TelemetryFrame_fields, &message);

    if (!status) {
        // encoding failed
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    // 3. 发送 (buffer, stream.bytes_written)
    mqtt_publish("fsae/telemetry", buffer, stream.bytes_written);
}

```
如果未来发现流量过大，可以将 `.proto` 中的 `float` 改为 `int32`，单位使用**毫伏(mV)** 和 **0.1摄氏度**。这样配合 Protobuf 的 Varint 编码（小整数占用字节少），可以压缩一半左右的体积。但目前的 `float` 方案实现最简单，建议先用着。

### 下一步建议

为了避免栈溢出，车端编码缓冲和 `TelemetryFrame` 最好使用静态存储，而不是在较小栈上定义大对象。当前 `CAN2RS485` 工程已经按这个原则实现，后续新增大数组或大结构时也应保持一致。
