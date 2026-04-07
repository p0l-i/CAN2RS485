# FSAE 遥测系统参考文档

BITFSAE 赛车遥测项目。车载 STM32 读取 CAN 数据，用 **Nanopb**（Protobuf 轻量实现）序列化后经 RS485 送入 DTU，由 DTU 通过 **MQTT** 无线上传至阿里云服务器，Grafana 实时可视化。

```
STM32(CAN) --RS485--> DTU --MQTT/4G--> 阿里云
                                          |
                               Mosquitto -> Telegraf -> InfluxDB -> Grafana
```

本目录只保存 Protobuf、DTU、服务器链路参考资料。车端实现以仓库根目录文档和源码为准。

---

## 目录

- `stm32_code/`
  - `.proto`、`.options`、Nanopb 生成结果
- `server_config/`
  - 服务器配置快照
- `local_sim2.py`
  - 本地模拟发送脚本
- `README_local_sim.md`
  - 模拟脚本使用说明
- `DTU_DEBUG_GUIDE.md`
  - DTU 联调说明
- `PROTO_GUIDE.md`
  - Protobuf 维护约束
- `STM32_GUIDE.md`
  - STM32 集成约束
- `服务器文件.md`
  - 服务器端配置说明

## 关键约束

- MQTT Topic 固定为 `fsae/telemetry`
- 基础遥测和 `modules` 共用同一条 `TelemetryFrame`
- 修改 `.proto` 字段名、类型或字段 ID 时，必须同步检查：
  - `stm32_code/`
  - `server_config/telegraf/telegraf.conf`
  - 车端工程 `Middlewares/Third_Party/nanopb/`

---

## 快速上手

**测试服务器链路（纯 MQTT）：**
```bash
pip install paho-mqtt protobuf pyserial
python local_sim2.py
```

**联调 DTU（USB 转 485）：**
```bash
python local_sim2.py --mode serial --serial-port COM3 --baudrate 115200
```

**同时验证两条链路：**
```bash
python local_sim2.py --mode both --serial-port COM3 --baudrate 115200
```

服务器地址：`123.57.174.98:1883`，Topic：`fsae/telemetry`，Grafana：`https://bitfsae.xin/monitor/`
