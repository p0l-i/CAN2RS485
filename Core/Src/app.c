#include "app.h"

#include <string.h>

#include "can.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"

#include "fsae_telemetry.pb.h"
#include "pb_encode.h"

#define APP_MODULE_COUNT                6U
#define APP_CELLS_PER_MODULE            23U
#define APP_TEMPS_PER_MODULE            8U
#define APP_TOTAL_CELL_COUNT            (APP_MODULE_COUNT * APP_CELLS_PER_MODULE)
#define APP_TOTAL_TEMP_COUNT            (APP_MODULE_COUNT * APP_TEMPS_PER_MODULE)

#define APP_CAN1_FILTER_BANK            0U
#define APP_CAN2_FILTER_BANK            14U
#define APP_CAN_SLAVE_START_BANK        14U

#define APP_CAN1_BASE_VOLTAGE_ID        0x180050F3UL
#define APP_CAN1_BASE_TEMP_ID           0x184050F3UL
#define APP_CAN1_PACK_SUMMARY_ID        0x186050F4UL
#define APP_CAN1_CELL_EXTREMA_ID        0x186150F4UL
#define APP_CAN1_TEMP_EXTREMA_ID        0x186250F4UL
#define APP_CAN1_STATUS_ID              0x186350F4UL
#define APP_CAN1_ALARM_ID               0x187650F4UL
#define APP_CAN1_CELL_SUM_ID            0x186750F4UL
#define APP_CAN1_IMD_DIAG_ID            0x186850F4UL
#define APP_CAN1_TOOL_FAULT_RESET_ID    0x188350F5UL
#define APP_CAN1_TOOL_ADC_CAL_ID        0x18A050F5UL
#define APP_CAN1_TOOL_CURRENT_DIR_ID    0x18A150F5UL
#define APP_CAN1_TOOL_RTC_SET_ID        0x18A350F5UL
#define APP_CAN1_HALL_ID                0x03C0U

#define APP_CAN2_CHARGER_FB_ID          0x18FF50E5UL
#define APP_CAN2_POWER_STATUS_ID        0x0401U
#define APP_CAN2_DIAG_STATUS_ID         0x0402U

#define APP_HEARTBEAT_PERIOD_MS         500U
#define APP_BASE_TELEMETRY_PERIOD_MS    100U
#define APP_MODULE_TELEMETRY_PERIOD_MS  500U
#define APP_SNAPSHOT_TIMEOUT_MS         2000U
#define APP_RS485_TX_TIMEOUT_MS         200U

#define APP_HALL_FAULT_BIT              (1UL << 18)
#define APP_IMD_FAULT_BIT               (1UL << 19)

typedef enum
{
  APP_PROTOCOL_UNKNOWN = 0,
  APP_PROTOCOL_LEGACY,
  APP_PROTOCOL_MODERN
} AppProtocol;

typedef struct
{
  uint16_t cell_voltage_mv[APP_TOTAL_CELL_COUNT];
  int16_t cell_temp_deci_c[APP_TOTAL_TEMP_COUNT];
  uint8_t module_voltage_valid[APP_MODULE_COUNT];
  uint8_t module_temp_valid[APP_MODULE_COUNT];
  uint32_t module_voltage_updated_ms[APP_MODULE_COUNT];
  uint32_t module_temp_updated_ms[APP_MODULE_COUNT];

  uint16_t pack_voltage_deci_v;
  uint8_t battery_soc;
  uint8_t imd_signal;
  uint8_t battery_state;
  uint8_t battery_alarm_level;
  uint16_t summary_current_raw;
  uint32_t pack_summary_updated_ms;

  uint16_t cell_voltage_sum_deci_v;
  uint32_t cell_voltage_sum_updated_ms;
  uint8_t imd_diag_payload[8];
  uint32_t imd_diag_updated_ms;

  uint16_t max_cell_voltage_mv;
  uint16_t min_cell_voltage_mv;
  uint8_t max_cell_index_zero_based;
  uint8_t min_cell_index_zero_based;
  uint32_t cell_extrema_updated_ms;

  int16_t max_temp_deci_c;
  int16_t min_temp_deci_c;
  uint8_t max_temp_index_zero_based;
  uint8_t min_temp_index_zero_based;
  uint8_t cooling_control;
  uint32_t temp_extrema_updated_ms;

  uint8_t pos_relay_state;
  uint8_t neg_relay_state;
  uint8_t pre_relay_state;
  uint8_t charge_state;
  uint8_t charge_comm_state;
  uint16_t charge_request_voltage_deci_v;
  uint16_t charge_request_current_deci_a;
  uint16_t precharge_voltage_deci_v;
  uint32_t status_updated_ms;

  int32_t hall_current_ma;
  uint8_t hall_error;
  uint8_t hall_error_code;
  uint16_t hall_sensor_name;
  uint8_t hall_sw_version;
  uint32_t hall_updated_ms;

  uint16_t charger_fb_voltage_deci_v;
  uint16_t charger_fb_current_deci_a;
  uint8_t charger_fb_state;
  uint32_t charger_fb_updated_ms;

  uint16_t can2_pack_voltage_deci_v;
  uint16_t can2_pack_power_raw;
  uint16_t can2_pack_current_raw;
  uint8_t can2_soc;
  int16_t can2_max_temp_deci_c;
  uint32_t can2_power_updated_ms;

  uint8_t can2_battery_state;
  uint8_t can2_alarm_level;
  uint16_t can2_error_rom_low16;
  uint8_t slave_offline_mask;
  uint32_t can2_diag_updated_ms;

  uint32_t alarm_fault_code;
  uint8_t hall_fault_active;
  uint8_t imd_fault_active;
  uint32_t battery_fault_code;
  uint32_t alarm_updated_ms;

  AppProtocol protocol;
  uint8_t can1_seen;
  uint8_t can2_seen;
} AppTelemetryState;

static volatile AppTelemetryState g_app_state;
static AppTelemetryState g_tx_snapshot;
static fsae_TelemetryFrame g_tx_frame;
static uint8_t g_pb_buffer[fsae_TelemetryFrame_size];
static uint32_t g_frame_counter;
static uint32_t g_last_heartbeat_ms;
static uint32_t g_last_base_tx_ms;
static uint32_t g_last_module_tx_ms;

static uint16_t App_ReadBe16(const uint8_t *data);
static uint16_t App_ReadLe16(const uint8_t *data);
static int32_t App_ReadBe32Signed(const uint8_t *data);
static uint8_t App_IsFresh(uint32_t now, uint32_t updated_ms);
static void App_SetProtocol(AppProtocol protocol);
static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now);
static void App_UpdateFaultCode(void);
static HAL_StatusTypeDef App_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, uint32_t filter_bank);
static void App_ProcessCanRx(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *header, const uint8_t *data);
static void App_ProcessCan1Rx(const CAN_RxHeaderTypeDef *header, const uint8_t *data);
static void App_ProcessCan2Rx(const CAN_RxHeaderTypeDef *header, const uint8_t *data);
static void App_ProcessCan1Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
static void App_ProcessCan2Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
static void App_ProcessCan2Std(uint32_t std_id, const uint8_t *data, uint8_t dlc);
static void App_ProcessVoltageFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessTempFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessPackSummary(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessCellVoltageSum(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessImdDiag(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessCellExtrema(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessTempExtrema(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessStatusFrame(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessAlarmFrame(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessChargerFeedback(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessCan2PowerStatus(const uint8_t *data, uint8_t dlc, uint32_t now);
static void App_ProcessCan2DiagStatus(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint32_t App_BuildBatteryFaultCode(const uint8_t *data);
static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now);
static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now);
static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp);
static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no);
static void App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                               int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no);
static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules);
static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size);

void App_Init(void)
{
  memset((void *)&g_app_state, 0, sizeof(g_app_state));
  g_frame_counter = 0U;
  g_last_heartbeat_ms = HAL_GetTick();
  g_last_base_tx_ms = HAL_GetTick();
  g_last_module_tx_ms = HAL_GetTick();

  if (App_CAN_ConfigFilter(&hcan1, APP_CAN1_FILTER_BANK) != HAL_OK)
  {
    Error_Handler();
  }

  if (App_CAN_ConfigFilter(&hcan2, APP_CAN2_FILTER_BANK) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t include_modules;

  if ((uint32_t)(now - g_last_heartbeat_ms) >= APP_HEARTBEAT_PERIOD_MS)
  {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    g_last_heartbeat_ms = now;
  }

  if ((g_app_state.can1_seen == 0U) && (g_app_state.can2_seen == 0U))
  {
    return;
  }

  if ((uint32_t)(now - g_last_base_tx_ms) < APP_BASE_TELEMETRY_PERIOD_MS)
  {
    return;
  }

  include_modules = (uint8_t)(((uint32_t)(now - g_last_module_tx_ms) >= APP_MODULE_TELEMETRY_PERIOD_MS) ? 1U : 0U);
  if (App_SendTelemetry(now, include_modules) == HAL_OK)
  {
    g_last_base_tx_ms = now;
    if (include_modules != 0U)
    {
      g_last_module_tx_ms = now;
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      return;
    }

    App_ProcessCanRx(hcan, &header, data);
  }
}

static uint16_t App_ReadBe16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint16_t App_ReadLe16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int32_t App_ReadBe32Signed(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];
  return (int32_t)(value - 0x80000000UL);
}

static uint8_t App_IsFresh(uint32_t now, uint32_t updated_ms)
{
  return (updated_ms != 0U) && (((uint32_t)(now - updated_ms)) <= APP_SNAPSHOT_TIMEOUT_MS);
}

static void App_SetProtocol(AppProtocol protocol)
{
  if ((protocol != APP_PROTOCOL_UNKNOWN) && (g_app_state.protocol != protocol))
  {
    g_app_state.protocol = protocol;
  }
}

static void App_UpdateFaultCode(void)
{
  g_app_state.battery_fault_code = App_ComputeFaultCode((const AppTelemetryState *)&g_app_state, HAL_GetTick());
}

static HAL_StatusTypeDef App_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = APP_CAN_SLAVE_START_BANK;
  filter.FilterBank = filter_bank;

  return HAL_CAN_ConfigFilter(hcan, &filter);
}

static void App_ProcessCanRx(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  if (hcan->Instance == CAN1)
  {
    App_ProcessCan1Rx(header, data);
  }
  else if (hcan->Instance == CAN2)
  {
    App_ProcessCan2Rx(header, data);
  }
}

static void App_ProcessCan1Rx(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  if (header->IDE == CAN_ID_EXT)
  {
    App_ProcessCan1Ext(header->ExtId, data, header->DLC);
    return;
  }

  if ((header->IDE == CAN_ID_STD) && (header->StdId == APP_CAN1_HALL_ID) && (header->DLC >= 8U))
  {
    g_app_state.hall_current_ma = App_ReadBe32Signed(data);
    g_app_state.hall_error = (uint8_t)(data[4] & 0x01U);
    g_app_state.hall_error_code = (uint8_t)((data[4] >> 1) & 0x7FU);
    g_app_state.hall_sensor_name = App_ReadBe16(&data[5]);
    g_app_state.hall_sw_version = data[7];
    g_app_state.hall_updated_ms = HAL_GetTick();
    g_app_state.can1_seen = 1U;
    App_UpdateFaultCode();
  }
}

static void App_ProcessCan2Rx(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  if (header->IDE == CAN_ID_EXT)
  {
    App_ProcessCan2Ext(header->ExtId, data, header->DLC);
  }
  else if (header->IDE == CAN_ID_STD)
  {
    App_ProcessCan2Std(header->StdId, data, header->DLC);
  }
}

static void App_ProcessCan1Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
  uint32_t now = HAL_GetTick();

  if (((ext_id - APP_CAN1_BASE_VOLTAGE_ID) & 0xFFFFU) == 0U)
  {
    App_ProcessVoltageFrame(ext_id, data, dlc, now);
    return;
  }

  if (((ext_id - APP_CAN1_BASE_TEMP_ID) & 0xFFFFU) == 0U)
  {
    App_ProcessTempFrame(ext_id, data, dlc, now);
    return;
  }

  switch (ext_id)
  {
    case APP_CAN1_PACK_SUMMARY_ID:
      App_ProcessPackSummary(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_CELL_SUM_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      App_ProcessCellVoltageSum(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_IMD_DIAG_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      App_ProcessImdDiag(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_CELL_EXTREMA_ID:
      App_ProcessCellExtrema(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_TEMP_EXTREMA_ID:
      App_ProcessTempExtrema(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_STATUS_ID:
      App_ProcessStatusFrame(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_ALARM_ID:
      App_ProcessAlarmFrame(data, dlc, now);
      g_app_state.can1_seen = 1U;
      break;

    case APP_CAN1_TOOL_FAULT_RESET_ID:
    case APP_CAN1_TOOL_ADC_CAL_ID:
    case APP_CAN1_TOOL_CURRENT_DIR_ID:
    case APP_CAN1_TOOL_RTC_SET_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      break;

    default:
      break;
  }
}

static void App_ProcessCan2Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
  if (ext_id == APP_CAN2_CHARGER_FB_ID)
  {
    App_ProcessChargerFeedback(data, dlc, HAL_GetTick());
    g_app_state.can2_seen = 1U;
  }
}

static void App_ProcessCan2Std(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
  switch (std_id)
  {
    case APP_CAN2_POWER_STATUS_ID:
      App_ProcessCan2PowerStatus(data, dlc, HAL_GetTick());
      g_app_state.can2_seen = 1U;
      break;

    case APP_CAN2_DIAG_STATUS_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      App_ProcessCan2DiagStatus(data, dlc, HAL_GetTick());
      g_app_state.can2_seen = 1U;
      break;

    default:
      break;
  }
}

static void App_ProcessVoltageFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint32_t frame_no = (ext_id - APP_CAN1_BASE_VOLTAGE_ID) >> 16;
  uint32_t module_idx;
  uint32_t frame_idx;
  uint32_t cell_offset;
  uint32_t cell_count;
  uint32_t base_idx;
  uint32_t src_idx;
  uint32_t i;

  if ((dlc < 8U) || (frame_no >= 36U))
  {
    return;
  }

  module_idx = frame_no / 6U;
  frame_idx = frame_no % 6U;

  if (frame_idx == 0U)
  {
    cell_offset = 0U;
    cell_count = 3U;
    src_idx = 2U;
  }
  else
  {
    cell_offset = 3U + ((frame_idx - 1U) * 4U);
    cell_count = 4U;
    src_idx = 0U;
  }

  base_idx = (module_idx * APP_CELLS_PER_MODULE) + cell_offset;
  for (i = 0U; i < cell_count; ++i)
  {
    g_app_state.cell_voltage_mv[base_idx + i] = App_ReadLe16(&data[src_idx + (i * 2U)]);
  }

  g_app_state.module_voltage_valid[module_idx] |= (uint8_t)(1U << frame_idx);
  g_app_state.module_voltage_updated_ms[module_idx] = now;
  g_app_state.can1_seen = 1U;
}

static void App_ProcessTempFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint32_t module_idx = (ext_id - APP_CAN1_BASE_TEMP_ID) >> 16;
  uint32_t base_idx;
  uint32_t i;
  uint8_t temp_count = APP_TEMPS_PER_MODULE;
  uint8_t src_idx = 0U;

  if ((dlc < 8U) || (module_idx >= APP_MODULE_COUNT))
  {
    return;
  }

  base_idx = module_idx * APP_TEMPS_PER_MODULE;
  if ((g_app_state.protocol != APP_PROTOCOL_MODERN) &&
      ((module_idx == 2U) || (module_idx == 3U)))
  {
    temp_count = APP_TEMPS_PER_MODULE - 1U;
    src_idx = 1U;
  }

  g_app_state.module_temp_valid[module_idx] = 0U;
  for (i = 0U; i < temp_count; ++i)
  {
    g_app_state.cell_temp_deci_c[base_idx + i] = (int16_t)(((int16_t)data[src_idx + i] - 30) * 10);
    g_app_state.module_temp_valid[module_idx] |= (uint8_t)(1U << i);
  }

  for (; i < APP_TEMPS_PER_MODULE; ++i)
  {
    g_app_state.cell_temp_deci_c[base_idx + i] = 0;
  }

  g_app_state.module_temp_updated_ms[module_idx] = now;
  g_app_state.can1_seen = 1U;
}

static void App_ProcessPackSummary(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 7U)
  {
    return;
  }

  g_app_state.pack_voltage_deci_v = App_ReadBe16(&data[0]);
  g_app_state.summary_current_raw = App_ReadBe16(&data[2]);
  g_app_state.battery_soc = data[4];
  g_app_state.imd_signal = data[5];
  g_app_state.battery_state = (uint8_t)((data[6] >> 4) & 0x0FU);
  g_app_state.battery_alarm_level = (uint8_t)(data[6] & 0x0FU);
  g_app_state.pack_summary_updated_ms = now;
}

static void App_ProcessCellVoltageSum(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 2U)
  {
    return;
  }

  g_app_state.cell_voltage_sum_deci_v = App_ReadBe16(&data[0]);
  g_app_state.cell_voltage_sum_updated_ms = now;
}

static void App_ProcessImdDiag(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return;
  }

  memcpy((void *)g_app_state.imd_diag_payload, data, 8U);
  g_app_state.imd_diag_updated_ms = now;
}

static void App_ProcessCellExtrema(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return;
  }

  g_app_state.max_cell_voltage_mv = App_ReadBe16(&data[0]);
  g_app_state.min_cell_voltage_mv = App_ReadBe16(&data[2]);
  g_app_state.max_cell_index_zero_based = data[4];
  g_app_state.min_cell_index_zero_based = data[5];
  g_app_state.cell_extrema_updated_ms = now;
}

static void App_ProcessTempExtrema(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 5U)
  {
    return;
  }

  g_app_state.max_temp_deci_c = (int16_t)(((int16_t)data[0] - 30) * 10);
  g_app_state.min_temp_deci_c = (int16_t)(((int16_t)data[1] - 30) * 10);
  g_app_state.max_temp_index_zero_based = data[2];
  g_app_state.min_temp_index_zero_based = data[3];
  g_app_state.cooling_control = data[4];
  g_app_state.temp_extrema_updated_ms = now;
}

static void App_ProcessStatusFrame(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return;
  }

  g_app_state.pos_relay_state = (uint8_t)((data[0] >> 6) & 0x03U);
  g_app_state.neg_relay_state = (uint8_t)((data[0] >> 4) & 0x03U);
  g_app_state.pre_relay_state = (uint8_t)((data[0] >> 2) & 0x03U);
  g_app_state.charge_state = (uint8_t)((data[1] >> 4) & 0x0FU);
  g_app_state.charge_comm_state = (uint8_t)((data[1] >> 3) & 0x01U);
  g_app_state.charge_request_voltage_deci_v = App_ReadBe16(&data[2]);
  g_app_state.charge_request_current_deci_a = App_ReadBe16(&data[4]);
  g_app_state.precharge_voltage_deci_v = App_ReadBe16(&data[6]);
  g_app_state.status_updated_ms = now;
}

static void App_ProcessAlarmFrame(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return;
  }

  g_app_state.alarm_fault_code = App_BuildBatteryFaultCode(data) & ~(APP_HALL_FAULT_BIT | APP_IMD_FAULT_BIT);
  g_app_state.slave_offline_mask = (uint8_t)((((uint8_t)(data[4] >> 1) & 0x03U) << 4) |
                                             (((uint8_t)(data[5] >> 1) & 0x0FU)));
  g_app_state.alarm_updated_ms = now;
  App_UpdateFaultCode();
}

static void App_ProcessChargerFeedback(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 5U)
  {
    return;
  }

  g_app_state.charger_fb_voltage_deci_v = App_ReadBe16(&data[0]);
  g_app_state.charger_fb_current_deci_a = App_ReadBe16(&data[2]);
  g_app_state.charger_fb_state = data[4];
  g_app_state.charger_fb_updated_ms = now;
}

static void App_ProcessCan2PowerStatus(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return;
  }

  g_app_state.can2_pack_voltage_deci_v = App_ReadLe16(&data[0]);
  g_app_state.can2_pack_power_raw = App_ReadLe16(&data[2]);
  g_app_state.can2_pack_current_raw = App_ReadLe16(&data[4]);
  g_app_state.can2_soc = data[6];
  g_app_state.can2_max_temp_deci_c = (int16_t)(((int16_t)data[7] - 30) * 10);
  g_app_state.can2_power_updated_ms = now;
}

static void App_ProcessCan2DiagStatus(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return;
  }

  g_app_state.can2_battery_state = (uint8_t)((data[0] >> 4) & 0x0FU);
  g_app_state.can2_alarm_level = (uint8_t)(data[0] & 0x0FU);
  g_app_state.imd_fault_active = (uint8_t)((data[4] >> 2) & 0x01U);
  g_app_state.hall_fault_active = (uint8_t)((data[4] >> 4) & 0x01U);
  g_app_state.can2_error_rom_low16 = App_ReadLe16(&data[5]);
  g_app_state.slave_offline_mask = (uint8_t)(data[7] & 0x3FU);
  g_app_state.can2_diag_updated_ms = now;
  App_UpdateFaultCode();
}

static uint32_t App_BuildBatteryFaultCode(const uint8_t *data)
{
  uint32_t faults = 0U;

  if ((data[0] & 0xC0U) != 0U) faults |= (1UL << 0);
  if ((data[0] & 0x30U) != 0U) faults |= (1UL << 1);
  if ((data[0] & 0x0CU) != 0U) faults |= (1UL << 2);
  if ((data[0] & 0x03U) != 0U) faults |= (1UL << 3);
  if ((data[1] & 0x80U) != 0U) faults |= (1UL << 4);
  if ((data[1] & 0x40U) != 0U) faults |= (1UL << 5);
  if ((data[1] & 0x30U) != 0U) faults |= (1UL << 6);
  if ((data[1] & 0x0CU) != 0U) faults |= (1UL << 7);
  if ((data[1] & 0x03U) != 0U) faults |= (1UL << 8);
  if ((data[2] & 0xC0U) != 0U) faults |= (1UL << 9);
  if ((data[2] & 0x30U) != 0U) faults |= (1UL << 10);
  if ((data[2] & 0x0CU) != 0U) faults |= (1UL << 11);
  if (((data[2] & 0x03U) != 0U) || ((data[3] & 0x30U) != 0U)) faults |= (1UL << 12);
  if (((data[3] & 0xC0U) != 0U) || ((data[3] & 0x0CU) != 0U)) faults |= (1UL << 13);
  if ((data[4] & 0x10U) != 0U) faults |= (1UL << 14);
  if ((data[4] & 0x08U) != 0U) faults |= (1UL << 15);
  if (((data[4] & 0x06U) != 0U) || ((data[5] & 0x20U) != 0U)) faults |= (1UL << 16);
  if ((data[5] & 0x80U) != 0U) faults |= (1UL << 17);
  if ((data[5] & 0x40U) != 0U) faults |= APP_HALL_FAULT_BIT;

  return faults;
}

static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now)
{
  uint32_t faults = 0U;

  if (App_IsFresh(now, state->alarm_updated_ms) != 0U)
  {
    faults |= state->alarm_fault_code;
  }

  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error != 0U))
  {
    faults |= APP_HALL_FAULT_BIT;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    if (state->hall_fault_active != 0U)
    {
      faults |= APP_HALL_FAULT_BIT;
    }
    if (state->imd_fault_active != 0U)
    {
      faults |= APP_IMD_FAULT_BIT;
    }
    faults |= (uint32_t)state->can2_error_rom_low16;
  }

  return faults;
}

static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now)
{
  uint32_t i;
  uint32_t total_mv = 0U;

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return (float)state->pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->cell_voltage_sum_updated_ms) != 0U)
  {
    return (float)state->cell_voltage_sum_deci_v / 10.0f;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != 0x3FU)
    {
      return 0.0f;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    total_mv += state->cell_voltage_mv[i];
  }

  return (float)total_mv / 1000.0f;
}

static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error == 0U))
  {
    return (float)state->hall_current_ma / 1000.0f;
  }

  if (App_IsFresh(now, state->charger_fb_updated_ms) != 0U)
  {
    return (float)state->charger_fb_current_deci_a / 10.0f;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    if (App_IsFresh(now, state->status_updated_ms) != 0U && (state->charge_state != 0U))
    {
      return (float)state->summary_current_raw / 10.0f;
    }

    return ((float)state->summary_current_raw - 10000.0f) / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return ((float)state->can2_pack_current_raw - 10000.0f) / 10.0f;
  }

  return 0.0f;
}

static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_soc;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return state->can2_soc;
  }

  return 0U;
}

static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->status_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    return state->can2_battery_state;
  }

  return 0U;
}

static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now)
{
  return (App_GetVcuStatus(state, now) == 5U) ? 1U : 0U;
}

static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now)
{
  return App_ComputeFaultCode(state, now);
}

static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp)
{
  if (max_temp != 0)
  {
    return (float)max_temp / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_max_temp_deci_c / 10.0f;
  }

  return 0.0f;
}

static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t i;
  uint16_t local_max = 0U;
  uint16_t local_min = 0xFFFFU;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;

  if (App_IsFresh(now, state->cell_extrema_updated_ms) != 0U)
  {
    *max_mv = state->max_cell_voltage_mv;
    *min_mv = state->min_cell_voltage_mv;
    *max_no = (uint32_t)state->max_cell_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_cell_index_zero_based + 1U;
    return;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != 0x3FU)
    {
      *max_mv = 0U;
      *min_mv = 0U;
      *max_no = 0U;
      *min_no = 0U;
      return;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    if (state->cell_voltage_mv[i] >= local_max)
    {
      local_max = state->cell_voltage_mv[i];
      local_max_no = i + 1U;
    }
    if (state->cell_voltage_mv[i] <= local_min)
    {
      local_min = state->cell_voltage_mv[i];
      local_min_no = i + 1U;
    }
  }

  *max_mv = local_max;
  *min_mv = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
}

static void App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                               int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t module_idx;
  uint32_t temp_idx;
  int16_t local_max = INT16_MIN;
  int16_t local_min = INT16_MAX;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;
  uint8_t has_valid = 0U;

  if (App_IsFresh(now, state->temp_extrema_updated_ms) != 0U)
  {
    *max_temp = state->max_temp_deci_c;
    *min_temp = state->min_temp_deci_c;
    *max_no = (uint32_t)state->max_temp_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_temp_index_zero_based + 1U;
    return;
  }

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t temp_base = module_idx * APP_TEMPS_PER_MODULE;

    if (App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_temp_valid[module_idx] == 0U)
    {
      *max_temp = 0;
      *min_temp = 0;
      *max_no = 0U;
      *min_no = 0U;
      return;
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) == 0U)
      {
        continue;
      }

      if (state->cell_temp_deci_c[temp_base + temp_idx] >= local_max)
      {
        local_max = state->cell_temp_deci_c[temp_base + temp_idx];
        local_max_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }
      if (state->cell_temp_deci_c[temp_base + temp_idx] <= local_min)
      {
        local_min = state->cell_temp_deci_c[temp_base + temp_idx];
        local_min_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }

      has_valid = 1U;
    }
  }

  if (has_valid == 0U)
  {
    *max_temp = 0;
    *min_temp = 0;
    *max_no = 0U;
    *min_no = 0U;
    return;
  }

  *max_temp = local_max;
  *min_temp = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
}

static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  uint32_t module_idx;

  frame->modules_count = 0U;

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t cell_base;
    uint32_t temp_base;
    uint32_t cell_idx;
    uint32_t temp_idx;
    uint32_t *voltage_field;
    int32_t *temp_field;

    if (App_IsFresh(now, state->module_voltage_updated_ms[module_idx]) == 0U ||
        App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_voltage_valid[module_idx] != 0x3FU ||
        state->module_temp_valid[module_idx] == 0U)
    {
      continue;
    }

    frame->modules[frame->modules_count] = (fsae_BatteryModule)fsae_BatteryModule_init_zero;
    frame->modules[frame->modules_count].module_id = module_idx + 1U;

    voltage_field = &frame->modules[frame->modules_count].v01;
    temp_field = &frame->modules[frame->modules_count].t1;
    cell_base = module_idx * APP_CELLS_PER_MODULE;
    temp_base = module_idx * APP_TEMPS_PER_MODULE;

    for (cell_idx = 0U; cell_idx < APP_CELLS_PER_MODULE; ++cell_idx)
    {
      voltage_field[cell_idx] = state->cell_voltage_mv[cell_base + cell_idx];
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) != 0U)
      {
        temp_field[temp_idx] = state->cell_temp_deci_c[temp_base + temp_idx];
      }
    }

    frame->modules_count++;
  }

  return (frame->modules_count > 0U) ? 1U : 0U;
}

static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules)
{
  AppTelemetryState *snapshot = &g_tx_snapshot;
  fsae_TelemetryFrame *frame = &g_tx_frame;
  pb_ostream_t stream;
  uint32_t max_mv;
  uint32_t min_mv;
  uint32_t max_mv_no;
  uint32_t min_mv_no;
  int32_t max_temp;
  int32_t min_temp;
  uint32_t max_temp_no;
  uint32_t min_temp_no;

  __disable_irq();
  *snapshot = *((const AppTelemetryState *)&g_app_state);
  __enable_irq();

  *frame = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
  frame->timestamp_ms = now;
  frame->frame_id = ++g_frame_counter;
  frame->hv_voltage = App_GetHvVoltage(snapshot, now);
  frame->hv_current = App_GetHvCurrent(snapshot, now);
  frame->battery_soc = App_GetBatterySoc(snapshot, now);
  frame->ready_to_drive = App_GetReadyToDrive(snapshot, now);
  frame->vcu_status = App_GetVcuStatus(snapshot, now);
  frame->fault_code = App_GetFaultCode(snapshot, now);
  frame->battery_fault_code = frame->fault_code;

  App_GetCellExtrema(snapshot, now, &max_mv, &min_mv, &max_mv_no, &min_mv_no);
  App_GetTempExtrema(snapshot, now, &max_temp, &min_temp, &max_temp_no, &min_temp_no);

  frame->max_cell_voltage = max_mv;
  frame->min_cell_voltage = min_mv;
  frame->max_cell_voltage_no = max_mv_no;
  frame->min_cell_voltage_no = min_mv_no;
  frame->max_temp = max_temp;
  frame->min_temp = min_temp;
  frame->max_temp_no = max_temp_no;
  frame->min_temp_no = min_temp_no;
  frame->battery_temp_max = App_GetBatteryTempMax(snapshot, now, max_temp);

  if (include_modules != 0U)
  {
    (void)App_BuildModules(snapshot, now, frame);
  }

  stream = pb_ostream_from_buffer(g_pb_buffer, sizeof(g_pb_buffer));
  if (!pb_encode(&stream, fsae_TelemetryFrame_fields, frame))
  {
    return HAL_ERROR;
  }

  return App_RS485_Transmit(g_pb_buffer, (uint16_t)stream.bytes_written);
}

static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  uint32_t start_ms;

  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);

  status = HAL_UART_Transmit(&huart2, (uint8_t *)data, size, APP_RS485_TX_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
    return status;
  }

  start_ms = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) > APP_RS485_TX_TIMEOUT_MS)
    {
      HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
      return HAL_TIMEOUT;
    }
  }

  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
  return HAL_OK;
}
