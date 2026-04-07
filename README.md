# CAN2RS485

基于 `STM32F405RGT6` 的车载遥测转接板。

主链路：

`CAN -> TelemetryFrame -> USART2/RS485 -> DTU`

## 当前状态

- 已实现旧主控 `CAN1` 主链路。
- 已补 `CAN2` 被动监听：`0x18FF50E5`、`0x401`、`0x402`。
- 基础帧 100 ms，带 `modules` 的帧 500 ms。
- `USART2` 严格按 `RS485_DIR -> UART_Transmit -> 等待 TC -> 拉低 DIR` 时序发送。
- 代码默认兼容旧主控；检测到新主控专用帧后切换到新协议分支。

## 当前未完成项

- 未实现 `0x18A*`、`0x188350F5` 等新主控工具命令闭环。
- 未实现 CAN2 主动转发/发送，仅做被动监听与上云聚合。
- 未扩展新主控全部诊断字段到 `TelemetryFrame`。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## 参考文档

- [`DOC/其他信息.md`](./DOC/其他信息.md)
- [`DOC/todo.md`](./DOC/todo.md)
- [`REFERENCE/旧主控 CAN 通讯协议.md`](./REFERENCE/%E6%97%A7%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/新主控 CAN 通讯协议.md`](./REFERENCE/%E6%96%B0%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [`REFERENCE/protobuf-master/STM32_GUIDE.md`](./REFERENCE/protobuf-master/STM32_GUIDE.md)
