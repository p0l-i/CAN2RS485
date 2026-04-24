# AGENTS.md

## 目标

本项目运行在 `STM32F405RGT6` 上，作用是把车上 CAN 数据转换为通过 `USART2 + RS485` 发送给 DTU 的 `nanopb/Protobuf` 遥测帧。当前已实现的主链路是：旧主控 `CAN1` 数据输入，编码为 `fsae_TelemetryFrame`，经 `RS485` 发出。

AI 代理在本仓库内工作时，必须优先保持这条链路稳定，避免引入会破坏 CubeMX 再生、CAN 接收时序、485 发射时序或 protobuf 兼容性的改动。

## 代码分区

- `Core/Src/app.c`、`Core/Inc/app.h` 是业务主区。协议解析、状态聚合、protobuf 组帧、RS485 发送逻辑优先放这里。
- `Core/Src/main.c`、`Core/Src/can.c`、`Core/Src/usart.c`、`Core/Src/gpio.c`、`Core/Src/stm32f4xx_it.c` 属于 STM32CubeMX 生成文件。
- `CAN2RS485.ioc` 是硬件配置源头。外设、引脚、时钟、NVIC、波特率的根定义以它为准。
- `cmake/stm32cubemx/CMakeLists.txt` 与根 `CMakeLists.txt` 共同决定编译输入；新增业务源文件时，必须同步更新构建文件。
- `Middlewares/Third_Party/nanopb/` 下是第三方与生成代码，除非明确在更新 `.proto` 或升级 nanopb，否则不要手改生成结果。
- `Drivers/` 与 `startup_stm32f405xx.s` 默认不改；只有在明确硬件启动或底层驱动缺陷需要时才动。

## 修改边界

- 修改 CubeMX 生成文件时，只允许写在 `/* USER CODE BEGIN ... */` 和 `/* USER CODE END ... */` 区块内；不要改动区块外内容、函数签名、生成顺序或注释骨架。

## 当前协议与硬件事实

- 主 MCU：`STM32F405RGT6`，外部 `8 MHz` HSE，系统时钟 `168 MHz`。
- `CAN1`：`PB8/PB9`，当前配置 `250000` bps。
- `CAN2`：`PB12/PB13`，当前配置 `500000` bps。
- `USART2`：`PA2/PA3`，当前为 RS485 数据口，`115200 8N1`。
- `RS485_DIR`：`PA4`，发前拉高，确认最后一个停止位发完后再拉低。
- `LED_HEARTBEAT`：`PC13`，`App_Run()` 里每 `500 ms` 翻转。
- 当前只要收到 `CAN1` 或 `CAN2` 任一总线的有效业务数据，就允许开始周期上报；`CAN1` 可能接旧主控或新主控，`CAN2` 也可能单独接入。

## 业务规则

- CAN 接收入口在 `HAL_CAN_RxFifo0MsgPendingCallback()`；这里必须保持短小，只做取帧和解析，不做阻塞发送、不做大内存分配。
- `App_ProcessCanRx()` 同时处理扩展帧和标准帧；标准帧 `0x03C0` 是霍尔电流数据。
- 电池数据解析依赖固定 ID 映射：
  - 电压基 ID：`0x180050F3`
  - 温度基 ID：`0x184050F3`
  - 包摘要：`0x186050F4`
  - 电芯极值：`0x186150F4`
  - 温度极值：`0x186250F4`
  - 状态：`0x186350F4`
  - 报警：`0x187650F4`
- 模组规模是固定假设：`6` 个模组、每模组 `23` 节电芯、`8` 路温度。修改这些数字前，必须同时核对：
  - `app.c` 中数组大小与帧解析逻辑
  - `fsae_telemetry.pb.h/.c` 中 `BatteryModule` 结构
  - 编码后的帧大小是否仍小于 `fsae_TelemetryFrame_size`
- 鲜度窗口目前是 `2000 ms`。依赖快照超时的推导逻辑很多，修改前要审查高压总压、总流、极值、模块上传是否会退化为零值。
- `App_GetHvCurrent()` 对摘要电流有两种解释：充电状态和非充电状态转换公式不同。不要在不了解旧主控协议的前提下重写。

## RS485 与串口规则

- `App_RS485_Transmit()` 的时序要求是硬约束：先拉高 `RS485_DIR`，`HAL_UART_Transmit()` 完成后继续等待 `UART_FLAG_TC`，再拉低 `RS485_DIR`。
- 不要把 `TC` 等待删除、替换成 `TXE`，或在发送完成前提前拉低方向脚，否则最后一个字节可能丢失。
- 默认不要在中断里直接发 `USART2`。
- 如果改为 DMA 或中断发送，必须重新定义“何时拉低 `RS485_DIR`”并保证语义等价于“最后一个停止位发送完成”。

## CAN 规则

- 两路 CAN 过滤器当前都配置为全通配；`CAN1` 使用 bank `0`，`CAN2` 使用 bank `14`，`SlaveStartFilterBank` 为 `14`。修改过滤器时，必须同时保证双 CAN bank 分配合法。
- `CAN2` 当前虽然已初始化和开中断，但业务未实现完整转发。不要在没有协议说明的情况下假设 `CAN2` 数据格式与 `CAN1` 相同。
- `AutoRetransmission` 当前关闭。涉及发送或链路可靠性调整时，不要只看 HAL 默认值，必须说明为什么要改。

## Protobuf / nanopb 规则

- 遥测编码目标是 `fsae_TelemetryFrame`。
- `modules` 字段只有在较低频率窗口才带上，基础帧周期 `100 ms`，模块详情周期 `500 ms`；不要无条件全量上送，避免帧过大或发送占用过久。
- 修改 protobuf 结构时，必须同时更新生成代码并检查：
  - `g_pb_buffer` 是否仍足够
  - `pb_encode()` 是否可能失败
  - DTU/云端消费者是否还能兼容
- 不要手写与生成头文件不一致的字段序号、重复字段数量或初始化器。

## 风格与实现要求

- 使用 C11，保持现有 HAL 风格与命名风格，不要混入 C++ 写法。
- 新增业务函数优先使用 `static`，仅在跨文件需要时才导出到头文件。
- 不要引入堆分配、RTOS 依赖、异常复杂宏或隐藏控制流。
- 代码注释只写协议来源、单位、时序原因、边界条件；不要写描述性废话。
- 单位要明确：`mv`、`deci_v`、`deci_c`、`ma` 这些现有命名必须延续，避免混用。

## 构建与验证

- 首选 CMake 预设构建：`cmake --preset Debug`，然后 `cmake --build --preset Debug`。
- 工具链默认来自 `cmake/gcc-arm-none-eabi.cmake`，优先使用官方 Arm GNU Toolchain。

## 提交前自检

- 这次改动是否会被 CubeMX 再生覆盖。
- 这次改动是否改变了 CAN ID、字节序、单位或鲜度语义。
- 这次改动是否引入阻塞到中断上下文。
- 这次改动是否影响 `USART2` 的 485 发射完成判定。
- 这次改动是否需要同步更新 `DOC/` 文档。
