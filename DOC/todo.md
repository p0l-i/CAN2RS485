## todo 
- [x] `local_sim2.py` 已验证 DTU + 服务器链路。
- [x] 使用 STM32CUBEMX 自动生成代码。
- [x] CAN1 和 CAN2 接收过滤器初始化。
- [x] 实现旧主控 `CAN1 -> TelemetryFrame -> RS485` 主链路。
- [x] 增加 `CAN2` 被动监听：`0x18FF50E5`、`0x401`、`0x402`。
- [x] 修复发送路径栈占用、温度映射、故障字覆盖和 DLC 防御。
- [x] 为未来切换新主控预留协议分流入口。

## remaining
- [ ] 若未来要完整支持新主控工具链，还需实现 `0x18A*` / `0x188350F5` 等命令的业务语义和应答帧。
- [ ] 若未来要做 CAN2 主动对接 VCU/充电机，还需明确本板在车上是“旁路监听”还是“参与总线发送”。
- [ ] 若未来需要把 IMD 诊断、CellVoltCal、更多 CAN2 故障细节上云，需要同步修改 `.proto`、nanopb 生成代码和云端 `telegraf.conf`。
