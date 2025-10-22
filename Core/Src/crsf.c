#include "crsf.h"

#include <string.h>
#include <stdio.h>

#include "usart.h"

#if CRSF_DEBUG_TEST
extern void UartSendText(const char *text);
#endif

/* ==================== 协议常量 ==================== */

#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8U
#define CRSF_ADDRESS_BROADCAST          0x00U

#define CRSF_FRAME_SIZE_MAX             64U
#define CRSF_DMA_BUFFER_SIZE            CRSF_FRAME_SIZE_MAX
#define CRSF_FRAME_HEADER_SIZE          2U   /* Address + Length */
#define CRSF_RC_PAYLOAD_SIZE            22U
#define CRSF_LINK_PAYLOAD_SIZE          10U

#define CRSF_SIGNAL_TIMEOUT_MS          1000U

/* ==================== 静态数据 ==================== */

static volatile CRSF_Data_t          g_crsf_data;
static CRSF_Data_t                   g_crsf_snapshot;
static CRSF_LinkStatistics_t         g_link_statistics;
static CRSF_LinkStatistics_t         g_link_snapshot;

static uint8_t                       g_dma_buffer[CRSF_DMA_BUFFER_SIZE];
static uint8_t                       g_parser_buffer[CRSF_FRAME_SIZE_MAX];
static uint16_t                      g_parser_index       = 0U;
static uint16_t                      g_expected_length    = 0U;

static volatile uint32_t             g_last_frame_tick    = 0U;
static volatile uint32_t             g_dma_overrun_count  = 0U;
static volatile uint32_t             g_idle_interrupt_cnt = 0U;

static bool                          g_signal_valid_cached   = false;
static uint32_t                      g_signal_state_change_ms = 0U;

/* CRC8 查表 (多项式 0xD5) */
static const uint8_t s_crc8_table[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/* ==================== 内部工具函数 ==================== */

static inline uint32_t crsf_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void crsf_exit_critical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static uint8_t crsf_crc8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;
  for (uint16_t i = 0U; i < length; ++i)
  {
    crc = s_crc8_table[crc ^ data[i]];
  }
  return crc;
}

static void crsf_reset_parser(void)
{
  g_parser_index    = 0U;
  g_expected_length = 0U;
  memset(g_parser_buffer, 0, sizeof(g_parser_buffer));
}

static void crsf_restart_dma(UART_HandleTypeDef *huart)
{
  if (HAL_UART_Receive_DMA(huart, g_dma_buffer, sizeof(g_dma_buffer)) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

static void crsf_parse_rc_channels(const uint8_t *payload)
{
  uint16_t temp_channels[CRSF_CHANNEL_COUNT];

  temp_channels[0]  = (uint16_t)((payload[0]     | (payload[1]  << 8))                    & 0x07FFU);
  temp_channels[1]  = (uint16_t)(((payload[1]>>3)| (payload[2]  << 5))                    & 0x07FFU);
  temp_channels[2]  = (uint16_t)(((payload[2]>>6)| (payload[3]  << 2) | (payload[4]<<10)) & 0x07FFU);
  temp_channels[3]  = (uint16_t)(((payload[4]>>1)| (payload[5]  << 7))                    & 0x07FFU);
  temp_channels[4]  = (uint16_t)(((payload[5]>>4)| (payload[6]  << 4))                    & 0x07FFU);
  temp_channels[5]  = (uint16_t)(((payload[6]>>7)| (payload[7]  << 1) | (payload[8]<<9))  & 0x07FFU);
  temp_channels[6]  = (uint16_t)(((payload[8]>>2)| (payload[9]  << 6))                    & 0x07FFU);
  temp_channels[7]  = (uint16_t)(((payload[9]>>5)| (payload[10] << 3))                    & 0x07FFU);
  temp_channels[8]  = (uint16_t)((payload[11]    | (payload[12] << 8))                    & 0x07FFU);
  temp_channels[9]  = (uint16_t)(((payload[12]>>3)| (payload[13] << 5))                   & 0x07FFU);
  temp_channels[10] = (uint16_t)(((payload[13]>>6)| (payload[14] << 2) | (payload[15]<<10))& 0x07FFU);
  temp_channels[11] = (uint16_t)(((payload[15]>>1)| (payload[16] << 7))                   & 0x07FFU);
  temp_channels[12] = (uint16_t)(((payload[16]>>4)| (payload[17] << 4))                   & 0x07FFU);
  temp_channels[13] = (uint16_t)(((payload[17]>>7)| (payload[18] << 1) | (payload[19]<<9)) & 0x07FFU);
  temp_channels[14] = (uint16_t)(((payload[19]>>2)| (payload[20] << 6))                   & 0x07FFU);
  temp_channels[15] = (uint16_t)(((payload[20]>>5)| (payload[21] << 3))                   & 0x07FFU);

  uint32_t now_ms = HAL_GetTick();
  uint32_t primask = crsf_enter_critical();
  memcpy((void *)g_crsf_data.channels, temp_channels, sizeof(temp_channels));
  g_crsf_data.timestamp_ms = now_ms;
  g_crsf_data.link_active  = true;
  g_crsf_data.frame_counter++;
  crsf_exit_critical(primask);

  g_last_frame_tick = now_ms;
}

static void crsf_parse_link_statistics(const uint8_t *payload)
{
  g_link_statistics.uplink_rssi_ant1     = payload[0];
  g_link_statistics.uplink_rssi_ant2     = payload[1];
  g_link_statistics.uplink_link_quality  = payload[2];
  g_link_statistics.uplink_snr           = (int8_t)payload[3];
  g_link_statistics.active_antenna       = payload[4];
  g_link_statistics.rf_mode              = payload[5];
  g_link_statistics.uplink_tx_power      = payload[6];
  g_link_statistics.downlink_rssi        = payload[7];
  g_link_statistics.downlink_link_quality= payload[8];
  g_link_statistics.downlink_snr         = (int8_t)payload[9];

  uint32_t primask = crsf_enter_critical();
  g_crsf_data.link_quality = payload[2];
  g_crsf_data.rssi_dbm     = (int8_t)payload[0];
  crsf_exit_critical(primask);
}

static void crsf_process_frame(uint8_t *frame, uint16_t length)
{
  if ((frame == NULL) || (length < 5U))
  {
    return;
  }

  const uint8_t frame_length = frame[1];
  if (frame_length < 2U)
  {
    return;
  }

  const uint16_t expected_total = (uint16_t)frame_length + CRSF_FRAME_HEADER_SIZE;
  if (length < expected_total)
  {
    return;
  }

  const uint8_t received_crc   = frame[expected_total - 1U];
  const uint8_t calculated_crc = crsf_crc8(&frame[2], (uint16_t)frame_length - 1U);
  if (calculated_crc != received_crc)
  {
    uint32_t primask = crsf_enter_critical();
    g_crsf_data.frame_error_counter++;
    crsf_exit_critical(primask);
#if CRSF_DEBUG_TEST
    char err_buf[64];
    const int len = snprintf(
        err_buf,
        sizeof(err_buf),
        "[CRSF_ERR] CRC mismatch: RX=%02X calc=%02X\r\n",
        received_crc,
        calculated_crc);
    if ((len > 0) && (len < (int)sizeof(err_buf)))
    {
      UartSendText(err_buf);
    }
#endif
    return;
  }

  const uint8_t frame_type    = frame[2];
  const uint8_t *payload      = &frame[3];
  const uint16_t payload_size = (uint16_t)frame_length - 2U;

  switch (frame_type)
  {
    case 0x16U:  /* RC Channels */
      if (payload_size >= CRSF_RC_PAYLOAD_SIZE)
      {
        crsf_parse_rc_channels(payload);
      }
      break;

    case 0x14U:  /* Link Statistics */
      if (payload_size >= CRSF_LINK_PAYLOAD_SIZE)
      {
        crsf_parse_link_statistics(payload);
      }
      break;

    default:
      break;
  }
}

static void crsf_consume_bytes(const uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0U; i < length; ++i)
  {
    uint8_t byte = data[i];

    if (g_parser_index == 0U)
    {
      if ((byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) || (byte == CRSF_ADDRESS_BROADCAST))
      {
        g_parser_buffer[g_parser_index++] = byte;
      }
    }
    else if (g_parser_index == 1U)
    {
      g_parser_buffer[g_parser_index++] = byte;
      g_expected_length = (uint16_t)byte + CRSF_FRAME_HEADER_SIZE;
      if ((g_expected_length < 4U) || (g_expected_length > CRSF_FRAME_SIZE_MAX))
      {
        crsf_reset_parser();
      }
    }
    else
    {
      g_parser_buffer[g_parser_index++] = byte;

      if (g_parser_index >= g_expected_length)
      {
        crsf_process_frame(g_parser_buffer, g_parser_index);
        crsf_reset_parser();
      }
      else if (g_parser_index >= CRSF_FRAME_SIZE_MAX)
      {
        crsf_reset_parser();
      }
    }
  }
}

/* ==================== 公共接口 ==================== */

void CRSF_Init(void)
{
  memset((void *)&g_crsf_data, 0, sizeof(g_crsf_data));
  for (uint8_t i = 0U; i < CRSF_CHANNEL_COUNT; ++i)
  {
    g_crsf_data.channels[i] = CRSF_CHANNEL_VALUE_NEUTRAL;
  }
  memset((void *)&g_link_statistics, 0, sizeof(g_link_statistics));
  g_crsf_data.link_active = false;
  g_last_frame_tick       = HAL_GetTick();
  g_dma_overrun_count     = 0U;
  g_idle_interrupt_cnt    = 0U;
  g_signal_valid_cached   = false;
  g_signal_state_change_ms = 0U;

  crsf_reset_parser();

  if (HAL_UART_Receive_DMA(&huart2, g_dma_buffer, sizeof(g_dma_buffer)) != HAL_OK)
  {
#if CRSF_DEBUG_TEST
    UartSendText("[CRSF] DMA start failed\r\n");
#endif
    Error_Handler();
  }
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

#if CRSF_DEBUG_TEST
  UartSendText("[CRSF] DMA receiver ready\r\n");
#endif
}

void CRSF_ProcessByte(uint8_t byte)
{
  crsf_consume_bytes(&byte, 1U);
}

void CRSF_Update(void)
{
  const uint32_t now = HAL_GetTick();

  uint32_t primask = crsf_enter_critical();
  const bool link_active = g_crsf_data.link_active;
  const uint32_t last_tick = g_crsf_data.timestamp_ms;
  crsf_exit_critical(primask);

  if (link_active && ((now - last_tick) > CRSF_SIGNAL_TIMEOUT_MS))
  {
    primask = crsf_enter_critical();
    g_crsf_data.link_active = false;
    crsf_exit_critical(primask);
  }

  (void)CRSF_IsSignalValid();
}

bool CRSF_IsLinkActive(void)
{
  uint32_t primask = crsf_enter_critical();
  const bool active = g_crsf_data.link_active;
  crsf_exit_critical(primask);
  return active;
}

bool CRSF_IsSignalValid(void)
{
  const uint32_t now = HAL_GetTick();

  uint32_t primask = crsf_enter_critical();
  const uint32_t last_update = g_crsf_data.timestamp_ms;
  const bool link_flag = g_crsf_data.link_active;
  crsf_exit_critical(primask);

  const bool timeout = ((now - last_update) > CRSF_SIGNAL_TIMEOUT_MS);
  const bool valid = link_flag && !timeout;

  if (valid != g_signal_valid_cached)
  {
    const uint32_t elapsed = now - last_update;
    if (valid)
    {
#if CRSF_DEBUG_TEST
      char msg[80];
      const int len = snprintf(msg, sizeof(msg),
                               "[CRSF] 信号恢复，离线%lu毫秒\r\n",
                               (unsigned long)((g_signal_state_change_ms == 0U) ? 0U : elapsed));
      if ((len > 0) && (len < (int)sizeof(msg)))
      {
        UartSendText(msg);
      }
#endif
      g_signal_state_change_ms = now;
    }
    else
    {
#if CRSF_DEBUG_TEST
      char msg[80];
      const int len = snprintf(msg, sizeof(msg),
                               "[CRSF] 信号丢失，超过%lu毫秒未收到数据\r\n",
                               (unsigned long)elapsed);
      if ((len > 0) && (len < (int)sizeof(msg)))
      {
        UartSendText(msg);
      }
#endif
      g_signal_state_change_ms = now;
    }
    g_signal_valid_cached = valid;
  }

  return valid;
}

void CRSF_GetData(CRSF_Data_t *dest)
{
  if (dest == NULL)
  {
    return;
  }

  uint32_t primask = crsf_enter_critical();
  g_crsf_snapshot = g_crsf_data;
  crsf_exit_critical(primask);

  *dest = g_crsf_snapshot;
}

bool CRSF_PullLatest(CRSF_Data_t *dest)
{
  if (dest == NULL)
  {
    return false;
  }

  static uint32_t last_published = 0U;
  bool has_new_frame = false;

  uint32_t primask = crsf_enter_critical();
  if (g_crsf_data.frame_counter != last_published)
  {
    g_crsf_snapshot = g_crsf_data;
    last_published = g_crsf_data.frame_counter;
    has_new_frame = true;
  }
  crsf_exit_critical(primask);

  if (has_new_frame)
  {
    *dest = g_crsf_snapshot;
  }
  return has_new_frame;
}

uint32_t CRSF_GetRxInterruptCount(void)
{
  return g_idle_interrupt_cnt;
}

uint32_t CRSF_GetDmaOverrunCount(void)
{
  return g_dma_overrun_count;
}

void CRSF_GetLinkStatistics(CRSF_LinkStatistics_t *dest)
{
  if (dest == NULL)
  {
    return;
  }

  uint32_t primask = crsf_enter_critical();
  g_link_snapshot = g_link_statistics;
  crsf_exit_critical(primask);

  *dest = g_link_snapshot;
}

/* ==================== HAL 回调接口 ==================== */

void CRSF_UART_IdleCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart2)
  {
    return;
  }

  __HAL_UART_CLEAR_IDLEFLAG(huart);

  DMA_HandleTypeDef *hdma = huart->hdmarx;
  if (hdma == NULL)
  {
    return;
  }

  if (HAL_UART_DMAStop(huart) != HAL_OK)
  {
    Error_Handler();
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(hdma);
  const uint16_t received  = (uint16_t)(sizeof(g_dma_buffer) - remaining);

  if (received >= sizeof(g_dma_buffer))
  {
    g_dma_overrun_count++;
  }

  if (received > 0U)
  {
    g_idle_interrupt_cnt += received;
    crsf_consume_bytes(g_dma_buffer, received);
  }

  memset(g_dma_buffer, 0, sizeof(g_dma_buffer));
  crsf_restart_dma(huart);
}

void CRSF_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart2)
  {
    return;
  }

  (void)HAL_UART_DMAStop(huart);
  crsf_reset_parser();
  crsf_restart_dma(huart);

  uint32_t primask = crsf_enter_critical();
  g_crsf_data.frame_error_counter++;
  crsf_exit_critical(primask);
}
