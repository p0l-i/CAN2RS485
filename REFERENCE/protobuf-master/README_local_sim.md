# 本地模拟与测试指南 (local_sim2.py)

`local_sim2.py` 是一个用于测试服务器数据接收链路的 Python 脚本。它模拟赛车发送遥测数据（Telemetry）和电池管理系统数据（BMS）到 MQTT 服务器。

## 1. 环境准备

确保你已经安装了 Python 以及必要的依赖库。

```bash
pip install paho-mqtt protobuf pyserial
```

如果你使用的是较新的 Python 版本（例如 Python 3.14），不要固定安装旧版 `protobuf==4.25.3`，否则可能在导入 `fsae_telemetry_pb2.py` 时触发兼容性错误。此时保持 `protobuf` 为当前最新版即可。

注意：你需要确保当前目录下有 `fsae_telemetry_pb2.py`文件。运行 `protoc --python_out=. fsae_telemetry.proto`，可生成该文件。脚本现在兼容“旧 pb2”和“按新单 Topic 结构重新生成后的 pb2”两种形态；如果你要验证新增的嵌套字段，必须先按最新 `.proto` 重新生成。

## 2. 脚本功能

这个脚本的主要功能是：
1.  **连接 MQTT 服务器**：连接到配置的公网 IP 和端口。
2.  **模拟物理数据**：模拟车辆的加速、刹车、滑行状态，并生成相应的 RPM、电压、电流、温度、SOC、单体极值与故障码等数据，使其看起来像真实的赛车数据（有物理惯性，不是纯随机）。
3.  **发送数据**：
    *   `fsae/telemetry` Topic: 统一发送 `TelemetryFrame`（10Hz）。
    *   新版协议下，基础遥测放在 `TelemetryFrame.header`、`fast_telemetry`、`vehicle_state`、`thermal_summary`、`alarms`。
    *   `TelemetryFrame.modules` 中的 BMS 详细数据每 5 帧刷新一次（即约 2Hz），仍复用同一个 Topic 和旧的 `modules` 结构。
4.  **支持多种输出链路**：
    *   `--mode mqtt`: 直接发到服务器 MQTT
    *   `--mode serial`: 通过 USB 转 485 向 DTU 串口送 Protobuf 原始字节流
    *   `--mode both`: 同时发 MQTT 和串口，适合联调

## 3. 配置修改

在 `local_sim2.py` 的头部，你可以修改服务器配置：

```python
# ================= 配置区域 =================
# 阿里云服务器的公网 IP
SERVER_IP = "123.57.174.98"   # 修改为你实际的服务器 IP
SERVER_PORT = 1883
TOPIC_TELEMETRY = "fsae/telemetry"
```

也可以直接用命令行参数覆盖，无需改文件。

## 4. 运行模拟

直接在终端运行：

```bash
python local_sim2.py
```

如果要测试 USB 转 485 + DTU：

```bash
python local_sim2.py --mode serial --serial-port COM3 --baudrate 115200
```

如果要一边喂 DTU，一边直接发 MQTT 对照：

```bash
python local_sim2.py --mode both --serial-port COM3 --baudrate 115200
```

如果 DTU 需要用特定包尾来切包，可以追加：

```bash
python local_sim2.py --mode serial --serial-port COM3 --packet-suffix-hex 0D0A
```

## 5. 常见问题

*   **缺少模块错误**：如果提示 `ModuleNotFoundError: No module named 'fsae_telemetry_pb2'`，说明你还没生成 Python 的 Protobuf 库文件。请运行 `protoc --python_out=. fsae_telemetry.proto`（确保你安装了 protoc 编译器）。
*   **看不到新增字段**：如果脚本还能正常发送，但服务器只收到旧字段，通常是因为本地 `fsae_telemetry_pb2.py` 还是旧版本，需要用最新 `.proto` 重新生成。
*   **Protobuf 导入时报错**：如果安装完依赖后，在导入 `fsae_telemetry_pb2` 或 `google.protobuf` 时仍报错，并且你的 Python 版本较新（如 3.14），优先升级 `protobuf` 到最新版，而不是固定到旧版。
*   **连接失败**：检查 `SERVER_IP` 是否正确，以及服务器的 1883 端口是否开放（防火墙/安全组）。
*   **串口打不开**：确认 USB 转 485 的串口号是否正确（设备管理器里查看），并关闭其他占用该串口的软件。
*   **DTU 收不到完整包**：优先检查 DTU 串口参数是否和脚本一致；如果 DTU 依赖包尾切包，再用 `--packet-suffix-hex` 增加结束符。
