import argparse
import random
import time

import paho.mqtt.client as mqtt

import fsae_telemetry_pb2 as pb  # 导入刚才生成的库

try:
    import can
except ImportError:
    can = None

try:
    import serial
except ImportError:
    serial = None

# ================= 配置区域 =================
# 阿里云服务器的公网 IP
SERVER_IP = "123.57.174.98" 
SERVER_PORT = 1883
TOPIC_TELEMETRY = "fsae/telemetry"
SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 115200
SERIAL_BYTESIZE = 8
SERIAL_STOPBITS = 1
SERIAL_PARITY = "N"
SERIAL_TIMEOUT = 1.0
SERIAL_SUFFIX_HEX = ""
PCAN_CHANNEL = "PCAN_USBBUS1"
PCAN_BITRATE = 250000

# 发送频率设置
BASE_FREQ = 10.0           # 基础频率 10Hz
LOOP_INTERVAL = 1.0 / BASE_FREQ 
BMS_DIVIDER = 5            # 10Hz / 5 = 2Hz

# ===========================================

START_TIMESTAMP = time.time()
LEGACY_CAN_SUMMARY_DIVIDER = 5
LEGACY_CAN_VOLTAGE_BASE_ID = 0x180050F3
LEGACY_CAN_TEMP_BASE_ID = 0x184050F3
LEGACY_CAN_PACK_SUMMARY_ID = 0x186050F4
LEGACY_CAN_CELL_EXTREMA_ID = 0x186150F4
LEGACY_CAN_TEMP_EXTREMA_ID = 0x186250F4
LEGACY_CAN_STATUS_ID = 0x186350F4
LEGACY_CAN_ALARM_ID = 0x187650F4
LEGACY_CAN_HALL_ID = 0x03C0


def enum_value(name, default_value):
    return getattr(pb, name, default_value)


def clamp_u8(value):
    return max(0, min(0xFF, int(value)))


def clamp_u16(value):
    return max(0, min(0xFFFF, int(value)))


def encode_be16(value):
    value = clamp_u16(value)
    return [(value >> 8) & 0xFF, value & 0xFF]


def encode_le16(value):
    value = clamp_u16(value)
    return [value & 0xFF, (value >> 8) & 0xFF]


def encode_legacy_temp_byte(temp_deci_c):
    temp_c = int(round(temp_deci_c / 10.0))
    return clamp_u8(temp_c + 30)


def encode_hall_current_raw(current_ma):
    value = (int(current_ma) + 0x80000000) & 0xFFFFFFFF
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


def mode_targets(mode):
    mapping = {
        "mqtt": {"mqtt"},
        "serial": {"serial"},
        "both": {"mqtt", "serial"},
        "pcan": {"pcan"},
        "mqtt+pcan": {"mqtt", "pcan"},
        "serial+pcan": {"serial", "pcan"},
        "all": {"mqtt", "serial", "pcan"},
    }
    return mapping[mode]


def get_current_time_ms():
    # 2. 修改这里：计算当前时间与启动时间的差值 (模拟单片机的 HAL_GetTick)
    diff = time.time() - START_TIMESTAMP
    return int(diff * 1000)

class CarSimulator:
    def __init__(self):
        # 初始化车辆状态 (用于模拟连续变化的数值)
        self.rpm = 0
        self.speed = 0
        self.apps = 0 # 油门
        self.brake = 0 # 刹车
        self.hv_voltage = 380.0
        self.hv_current = 0
        self.motor_temp = 40.0
        self.state = "IDLE" # IDLE, ACCEL, BRAKE, COAST
        self.frame_count = 0
        self.module_snapshots = []

    def _build_alarm_specs(self, max_temp, min_voltage):
        alarms = []
        if min_voltage < 3900:
            alarms.append((1001, enum_value("ALARM_SEVERITY_WARNING", 2), "min cell voltage low"))
        if max_temp > 500:
            alarms.append((1002, enum_value("ALARM_SEVERITY_ERROR", 3), "battery over temperature"))
        if self.hv_current < -10:
            alarms.append((1003, enum_value("ALARM_SEVERITY_INFO", 1), "regen current active"))
        if not alarms:
            alarms.append((1000, enum_value("ALARM_SEVERITY_INFO", 1), "system nominal"))
        return alarms

    def _populate_v2_fields(self, frame, timestamp_ms, soc_pct, max_voltage, min_voltage,
                            max_voltage_index, min_voltage_index, max_temp, min_temp,
                            max_temp_index, min_temp_index):
        frame.header.timestamp_ms = timestamp_ms
        frame.header.seq = self.frame_count
        frame.header.source_id = 1

        frame.fast_telemetry.hv_voltage_dv = int(round(self.hv_voltage * 10.0))
        frame.fast_telemetry.hv_current_ma = int(round(self.hv_current * 1000.0))
        frame.fast_telemetry.battery_temp_max_dc = max_temp
        frame.fast_telemetry.driving_mode = enum_value("DRIVING_MODE_DRIVE", 3) if self.rpm > 0 else enum_value("DRIVING_MODE_READY", 2)
        frame.fast_telemetry.speed_kmh = self.speed

        frame.vehicle_state.speed_kmh = self.speed
        frame.vehicle_state.driving_mode = enum_value("DRIVING_MODE_DRIVE", 3) if self.rpm > 0 else enum_value("DRIVING_MODE_READY", 2)
        frame.vehicle_state.throttle_position = self.apps
        frame.vehicle_state.brake_position = self.brake
        frame.vehicle_state.ready_to_drive = (self.rpm > 0)
        frame.vehicle_state.vcu_status = enum_value("VCU_STATUS_HV_ENABLED", 3) if self.rpm > 0 else enum_value("VCU_STATUS_OFF", 1)

        motor_specs = [
            ("MOTOR_POSITION_FRONT_LEFT", 1, 0.98, 1.02),
            ("MOTOR_POSITION_FRONT_RIGHT", 2, 1.00, 1.00),
            ("MOTOR_POSITION_REAR_LEFT", 3, 1.01, 0.99),
            ("MOTOR_POSITION_REAR_RIGHT", 4, 1.03, 0.97),
        ]
        for enum_name, default_pos, rpm_scale, temp_scale in motor_specs:
            motor = frame.vehicle_state.motors.add()
            motor.position = enum_value(enum_name, default_pos)
            motor.rpm = int(self.rpm * rpm_scale)
            motor.torque_nm = int(self.hv_current * 0.8 * rpm_scale)
            motor.power_w = int(self.hv_voltage * self.hv_current * rpm_scale)
            motor.motor_temp_dc = int(round(self.motor_temp * temp_scale * 10.0))
            motor.inverter_temp_dc = int(round((self.motor_temp - 5.0) * temp_scale * 10.0))
            motor.motor_error = 0

        sensor_specs = [
            ("MOTOR_POSITION_FRONT_LEFT", 1, 6200),
            ("MOTOR_POSITION_FRONT_RIGHT", 2, 6400),
            ("MOTOR_POSITION_REAR_LEFT", 3, 6800),
            ("MOTOR_POSITION_REAR_RIGHT", 4, 7000),
        ]
        for enum_name, default_pos, base_temp in sensor_specs:
            sensor = frame.thermal_summary.sensors.add()
            sensor.position = enum_value(enum_name, default_pos)
            sensor.min_temp_centi_c = base_temp - 180
            sensor.max_temp_centi_c = base_temp + 220
            sensor.avg_temp_centi_c = base_temp
            for chunk_index in range(4):
                chunk = sensor.chunks.add()
                chunk.position = sensor.position
                chunk.frame_id = self.frame_count
                chunk.chunk_index = chunk_index
                chunk.chunk_count = 4
                chunk.pixel_temp_centi_c = sensor.min_temp_centi_c + (chunk_index * 120)

        for alarm_id, severity, message in self._build_alarm_specs(max_temp, min_voltage):
            alarm = frame.alarms.add()
            alarm.alarm_id = alarm_id
            alarm.severity = severity
            alarm.message = message

        frame.battery_soc = soc_pct
        frame.max_cell_voltage = max_voltage
        frame.min_cell_voltage = min_voltage
        frame.max_cell_voltage_no = max_voltage_index
        frame.min_cell_voltage_no = min_voltage_index
        frame.max_temp = max_temp
        frame.min_temp = min_temp
        frame.max_temp_no = max_temp_index
        frame.min_temp_no = min_temp_index
        frame.battery_fault_code = 0
        for alarm_id, _, _ in self._build_alarm_specs(max_temp, min_voltage):
            if alarm_id == 1001:
                frame.battery_fault_code |= 0x01
            elif alarm_id == 1002:
                frame.battery_fault_code |= 0x02
            elif alarm_id == 1003:
                frame.battery_fault_code |= 0x04

    def _refresh_bms_cache(self):
        modules = []
        for i in range(6):
            base_vol = 4000 + (i * 10)
            voltages = [int(base_vol + random.randint(-15, 15)) for _ in range(23)]
            temps = [350 + i * 5 + random.randint(-5, 5) for _ in range(8)]
            modules.append({
                "module_id": i + 1,
                "voltages": voltages,
                "temps": temps,
            })
        self.module_snapshots = modules

    def update_physics(self):
        """模拟物理变化，让曲线看起来真实"""
        # 1. 随机切换驾驶状态
        if random.random() < 0.05: # 5%概率改变状态
            self.state = random.choice(["ACCEL", "BRAKE", "COAST", "ACCEL"])
        
        # 2. 根据状态更新数据
        if self.state == "ACCEL":
            self.apps = min(self.apps + 5, 100)
            self.brake = max(self.brake - 10, 0)
            self.rpm = min(self.rpm + 200 + random.randint(-50, 50), 12000)
            self.hv_current = (self.apps / 100) * 200 # 电流跟油门走
        
        elif self.state == "BRAKE":
            self.apps = max(self.apps - 10, 0)
            self.brake = min(self.brake + 10, 80)
            self.rpm = max(self.rpm - 400, 0)
            self.hv_current = -20 # 动能回收模拟
            
        elif self.state == "COAST":
            self.apps = max(self.apps - 5, 0)
            self.brake = 0
            self.rpm = max(self.rpm - 100, 0)
            self.hv_current = 5 # 待机电流

        # 3. 模拟温度缓慢上升
        if self.rpm > 5000:
            self.motor_temp += 0.05
        else:
            self.motor_temp = max(self.motor_temp - 0.02, 30)

        # 4. 电压随负载波动
        self.hv_voltage = 380.0 - (self.hv_current * 0.05) + random.uniform(-0.1, 0.1)
        self.speed = min(120, max(0, int(self.rpm / 90)))

    def generate_frame(self):
        """生成单 Topic 遥测帧；基础信息 10Hz，BMS 详细数据 2Hz 刷新一次"""
        self.update_physics()
        self.frame_count += 1
        include_bms = (self.frame_count % BMS_DIVIDER == 0)
        if include_bms or not self.module_snapshots:
            self._refresh_bms_cache()

        frame = pb.TelemetryFrame()
        
        timestamp_ms = get_current_time_ms()
        all_voltages = []
        all_temps = []
        for module_data in self.module_snapshots:
            all_voltages.extend(module_data["voltages"])
            all_temps.extend(module_data["temps"])

        max_voltage = max(all_voltages)
        min_voltage = min(all_voltages)
        max_voltage_index = all_voltages.index(max_voltage) + 1
        min_voltage_index = all_voltages.index(min_voltage) + 1
        max_temp = max(all_temps)
        min_temp = min(all_temps)
        max_temp_index = all_temps.index(max_temp) + 1
        min_temp_index = all_temps.index(min_temp) + 1

        soc_pct = max(0, min(100, int((self.hv_voltage - 320.0) / 0.7)))

        self._populate_v2_fields(frame, timestamp_ms, soc_pct, max_voltage, min_voltage,
                                 max_voltage_index, min_voltage_index, max_temp, min_temp,
                                 max_temp_index, min_temp_index)

        if include_bms:
            for module_data in self.module_snapshots:
                module = frame.modules.add()
                module.module_id = module_data["module_id"]
                for j, val in enumerate(module_data["voltages"], start=1):
                    setattr(module, f"v{j:02d}", val)
                for k, temp in enumerate(module_data["temps"], start=1):
                    setattr(module, f"t{k}", temp)

        return frame

    def build_legacy_can_messages(self):
        if not self.module_snapshots:
            self._refresh_bms_cache()

        messages = []
        all_voltages = []
        all_temps = []

        for module_data in self.module_snapshots:
            all_voltages.extend(module_data["voltages"])
            all_temps.extend(module_data["temps"])

        for module_index, module_data in enumerate(self.module_snapshots):
            voltages = module_data["voltages"]
            for frame_index in range(6):
                ext_id = LEGACY_CAN_VOLTAGE_BASE_ID + ((module_index * 6 + frame_index) << 16)
                if frame_index == 0:
                    payload = [0x00, 0x00]
                    payload.extend(encode_le16(voltages[0]))
                    payload.extend(encode_le16(voltages[1]))
                    payload.extend(encode_le16(voltages[2]))
                else:
                    start = 3 + ((frame_index - 1) * 4)
                    payload = []
                    for cell_index in range(start, start + 4):
                        payload.extend(encode_le16(voltages[cell_index]))
                messages.append((ext_id, payload, True))

            temp_payload = [encode_legacy_temp_byte(temp) for temp in module_data["temps"]]
            if module_index in (2, 3):
                temp_payload = [0x00] + temp_payload[:7]
            ext_id = LEGACY_CAN_TEMP_BASE_ID + (module_index << 16)
            messages.append((ext_id, temp_payload, True))

        if self.frame_count % LEGACY_CAN_SUMMARY_DIVIDER == 0:
            max_voltage = max(all_voltages)
            min_voltage = min(all_voltages)
            max_voltage_index = all_voltages.index(max_voltage)
            min_voltage_index = all_voltages.index(min_voltage)
            max_temp = max(all_temps)
            min_temp = min(all_temps)
            max_temp_index = all_temps.index(max_temp)
            min_temp_index = all_temps.index(min_temp)
            soc_pct = max(0, min(100, int((self.hv_voltage - 320.0) / 0.7)))

            battery_state = 5 if self.rpm > 0 else 3
            battery_alarm_level = 0
            alarm_payload = [0x00] * 6
            if min_voltage < 3900:
                battery_alarm_level = max(battery_alarm_level, 1)
                alarm_payload[0] |= 0x10
            if max_temp > 500:
                battery_alarm_level = max(battery_alarm_level, 2)
                alarm_payload[0] |= 0x08

            current_raw = clamp_u16(int(round(self.hv_current * 10.0)) + 10000)
            pack_voltage_dv = clamp_u16(int(round(self.hv_voltage * 10.0)))

            summary_payload = []
            summary_payload.extend(encode_be16(pack_voltage_dv))
            summary_payload.extend(encode_be16(current_raw))
            summary_payload.extend([
                clamp_u8(soc_pct),
                0x00,
                ((battery_state & 0x0F) << 4) | (battery_alarm_level & 0x0F),
            ])
            messages.append((LEGACY_CAN_PACK_SUMMARY_ID, summary_payload, True))

            cell_extrema_payload = []
            cell_extrema_payload.extend(encode_be16(max_voltage))
            cell_extrema_payload.extend(encode_be16(min_voltage))
            cell_extrema_payload.extend([
                clamp_u8(max_voltage_index),
                clamp_u8(min_voltage_index),
            ])
            messages.append((LEGACY_CAN_CELL_EXTREMA_ID, cell_extrema_payload, True))

            temp_extrema_payload = [
                encode_legacy_temp_byte(max_temp),
                encode_legacy_temp_byte(min_temp),
                clamp_u8(max_temp_index),
                clamp_u8(min_temp_index),
                0x00,
            ]
            messages.append((LEGACY_CAN_TEMP_EXTREMA_ID, temp_extrema_payload, True))

            relay_state = 1 if self.rpm > 0 else 0
            status_payload = [
                ((relay_state & 0x03) << 6) | ((relay_state & 0x03) << 4) | ((relay_state & 0x03) << 2),
                0x00,
            ]
            status_payload.extend(encode_be16(pack_voltage_dv))
            status_payload.extend(encode_be16(0))
            status_payload.extend(encode_be16(pack_voltage_dv))
            messages.append((LEGACY_CAN_STATUS_ID, status_payload, True))

            messages.append((LEGACY_CAN_ALARM_ID, alarm_payload, True))

        hall_payload = encode_hall_current_raw(int(round(self.hv_current * 1000.0)))
        hall_payload.extend([0x00, 0x12, 0x34, 0x01])
        messages.append((LEGACY_CAN_HALL_ID, hall_payload, False))

        return messages


def parse_args():
    parser = argparse.ArgumentParser(
        description="FSAE telemetry simulator: send protobuf frames via MQTT, RS-485 serial, or both."
    )
    parser.add_argument(
        "--mode",
        choices=["mqtt", "serial", "both", "pcan", "mqtt+pcan", "serial+pcan", "all"],
        default="mqtt",
        help=(
            "mqtt: direct to broker; serial: write protobuf to USB-RS485; both: send to both paths; "
            "pcan: emit legacy CAN1 frames through PCAN."
        ),
    )
    parser.add_argument("--server-ip", default=SERVER_IP)
    parser.add_argument("--server-port", type=int, default=SERVER_PORT)
    parser.add_argument("--topic", default=TOPIC_TELEMETRY)
    parser.add_argument("--serial-port", default=SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=SERIAL_BAUDRATE)
    parser.add_argument("--bytesize", type=int, default=SERIAL_BYTESIZE)
    parser.add_argument("--stopbits", type=float, default=SERIAL_STOPBITS)
    parser.add_argument("--parity", default=SERIAL_PARITY, choices=["N", "E", "O", "M", "S"])
    parser.add_argument("--serial-timeout", type=float, default=SERIAL_TIMEOUT)
    parser.add_argument(
        "--packet-suffix-hex",
        default=SERIAL_SUFFIX_HEX,
        help="Optional hex suffix appended to every serial packet, e.g. 0A or 0D0A.",
    )
    parser.add_argument("--pcan-channel", default=PCAN_CHANNEL, help="PCAN adapter channel, e.g. PCAN_USBBUS1.")
    parser.add_argument("--pcan-bitrate", type=int, default=PCAN_BITRATE, help="Legacy CAN1 bitrate, default 250000.")
    return parser.parse_args()


def open_mqtt_client(args):
    print(f"Connecting to MQTT Broker: {args.server_ip}:{args.server_port}...")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(args.server_ip, args.server_port, 60)
    print(f"MQTT connected. Topic: {args.topic}")
    return client


def open_serial_port(args):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
    print(
        f"Opening serial port: {args.serial_port} | {args.baudrate},{args.bytesize},{args.parity},{args.stopbits}"
    )
    ser = serial.Serial(
        port=args.serial_port,
        baudrate=args.baudrate,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.serial_timeout,
    )
    print("Serial connected.")
    return ser


def open_pcan_bus(args):
    if can is None:
        raise RuntimeError("python-can is not installed. Run: pip install python-can")
    print(f"Opening PCAN bus: {args.pcan_channel} | bitrate={args.pcan_bitrate}")
    bus = can.Bus(
        interface="pcan",
        channel=args.pcan_channel,
        bitrate=args.pcan_bitrate,
    )
    print("PCAN connected.")
    return bus


def build_serial_packet(frame, packet_suffix_hex):
    payload = frame.SerializeToString()
    if packet_suffix_hex:
        payload += bytes.fromhex(packet_suffix_hex)
    return payload


def get_frame_seq(frame):
    return frame.header.seq


def get_frame_timestamp(frame):
    return frame.header.timestamp_ms


def get_frame_rpm(frame):
    return frame.vehicle_state.motors[0].rpm if len(frame.vehicle_state.motors) > 0 else 0


def main():
    args = parse_args()
    client = None
    ser = None
    bus = None
    targets = mode_targets(args.mode)

    try:
        if "mqtt" in targets:
            client = open_mqtt_client(args)
        if "serial" in targets:
            ser = open_serial_port(args)
        if "pcan" in targets:
            bus = open_pcan_bus(args)
    except Exception as e:
        print(f"Initialization failed: {e}")
        return

    print("Starting simulation...")
    print(f"Mode: {args.mode} | Base Freq: {BASE_FREQ}Hz | BMS Freq: {BASE_FREQ/BMS_DIVIDER}Hz")

    sim = CarSimulator()

    try:
        while True:
            start_time = time.time()

            frame = sim.generate_frame()

            if client is not None:
                client.publish(args.topic, frame.SerializeToString())

            if ser is not None:
                serial_payload = build_serial_packet(frame, args.packet_suffix_hex)
                ser.write(serial_payload)
                ser.flush()

            can_count = 0
            if bus is not None:
                legacy_messages = sim.build_legacy_can_messages()
                can_count = len(legacy_messages)
                for arbitration_id, data, is_extended_id in legacy_messages:
                    bus.send(
                        can.Message(
                            arbitration_id=arbitration_id,
                            is_extended_id=is_extended_id,
                            data=data,
                        )
                    )

            if len(frame.modules) > 0:
                extra = ""
                if ser is not None:
                    extra = f" | serial_bytes: {len(serial_payload)}"
                if bus is not None:
                    extra += f" | can_frames: {can_count}"
                print(f"Sent merged telemetry+BMS frame @ {get_frame_timestamp(frame)}{extra}")

            # 打印日志 (每 10 帧打印一次，避免刷屏)
            if sim.frame_count % 10 == 0:
                pcan_text = f" | CAN: {can_count:02d}" if bus is not None else ""
                print(
                    f"ID: {get_frame_seq(frame):05d} | State: {sim.state:5s} | RPM: {get_frame_rpm(frame):5d} "
                    f"| SOC: {frame.battery_soc:3d}%{pcan_text}"
                )

            # 精确控制频率
            elapsed = time.time() - start_time
            sleep_time = max(0, LOOP_INTERVAL - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nSimulation stopped.")
        if client is not None:
            client.disconnect()
        if ser is not None and ser.is_open:
            ser.close()
        if bus is not None:
            bus.shutdown()

if __name__ == "__main__":
    main()
