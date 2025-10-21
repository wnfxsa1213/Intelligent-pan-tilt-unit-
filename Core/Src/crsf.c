/**
 ******************************************************************************
 * @file    crsf.c
 * @brief   CRSF协议解析实现
 * @version 1.0
 * @date    2025-01-22
 ******************************************************************************
 * @attention
 *
 * 这个憨批协议的帧结构：
 * [Address][Length][Type][Payload...][CRC]
 * CRC计算覆盖 Type+Payload，使用多项式0xD5。
 *
 ******************************************************************************
 */

#include "crsf.h"

#include <string.h>

#include "usart.h"

#define CRSF_DEVICE_ADDRESS              0xC8U
#define CRSF_FRAME_TYPE_RC_CHANNELS      0x16U
#define CRSF_MAX_FRAME_SIZE              64U
#define CRSF_PAYLOAD_MIN_LENGTH          2U

typedef enum
{
  CRSF_STATE_WAIT_ADDRESS = 0,
  CRSF_STATE_WAIT_LENGTH,
  CRSF_STATE_READ_PAYLOAD
} CRSF_ParserState_t;

static volatile CRSF_Data_t g_crsf_data;
static CRSF_ParserState_t   g_parser_state            = CRSF_STATE_WAIT_ADDRESS;
static uint8_t              g_frame_buffer[CRSF_MAX_FRAME_SIZE];
static uint8_t              g_expected_length          = 0U;
static uint8_t              g_payload_index            = 0U;
static uint8_t              g_uart_rx_byte             = 0U;
static volatile bool        g_restart_pending          = false;
static uint32_t             g_last_frame_tick          = 0U;
static uint32_t             g_last_restart_tick        = 0U;

static void crsf_reset_parser(void);
static void crsf_handle_frame(const uint8_t *frame, uint8_t length);
static void crsf_parse_rc_channels(const uint8_t *payload, uint8_t length);
static uint8_t crsf_crc8(const uint8_t *data, uint8_t length);

void CRSF_Init(void)
{
  memset((void *)&g_crsf_data, 0, sizeof(g_crsf_data));
  for (uint8_t i = 0U; i < CRSF_CHANNEL_COUNT; ++i)
  {
    g_crsf_data.channels[i] = 992U;  /* CRSF中立值 */
  }
  g_crsf_data.link_active = false;
  g_last_frame_tick       = HAL_GetTick();
  g_last_restart_tick     = g_last_frame_tick;

  crsf_reset_parser();

  if (HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U) != HAL_OK)
  {
    g_crsf_data.frame_error_counter++;
    g_restart_pending   = true;
    g_last_restart_tick = g_last_frame_tick;
  }
}

void CRSF_ProcessByte(uint8_t byte)
{
  switch (g_parser_state)
  {
    case CRSF_STATE_WAIT_ADDRESS:
      if (byte == CRSF_DEVICE_ADDRESS)
      {
        g_parser_state = CRSF_STATE_WAIT_LENGTH;
      }
      break;

    case CRSF_STATE_WAIT_LENGTH:
      if ((byte == 0U) || (byte > CRSF_MAX_FRAME_SIZE))
      {
        crsf_reset_parser();
        break;
      }
      g_expected_length = byte;
      g_payload_index   = 0U;
      g_parser_state    = CRSF_STATE_READ_PAYLOAD;
      break;

    case CRSF_STATE_READ_PAYLOAD:
      if (g_payload_index < CRSF_MAX_FRAME_SIZE)
      {
        g_frame_buffer[g_payload_index++] = byte;
      }

      if (g_payload_index >= g_expected_length)
      {
        crsf_handle_frame(g_frame_buffer, g_expected_length);
        crsf_reset_parser();
      }
      break;

    default:
      crsf_reset_parser();
      break;
  }
}

void CRSF_Update(void)
{
  const uint32_t now = HAL_GetTick();

  if (g_restart_pending)
  {
    if ((now - g_last_restart_tick) >= 10U)
    {
      if (HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U) == HAL_OK)
      {
        g_restart_pending = false;
      }
      else
      {
        g_crsf_data.frame_error_counter++;
        g_last_restart_tick = now;
      }
    }
  }

  if (g_crsf_data.link_active && ((now - g_last_frame_tick) > CRSF_TIMEOUT_MS))
  {
    g_crsf_data.link_active = false;
  }
}

bool CRSF_IsLinkActive(void)
{
  return g_crsf_data.link_active;
}

void CRSF_GetData(CRSF_Data_t *dest)
{
  if (dest == NULL)
  {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  CRSF_Data_t snapshot = g_crsf_data;
  __set_PRIMASK(primask);
  *dest = snapshot;
}

void CRSF_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  CRSF_ProcessByte(g_uart_rx_byte);

  if (HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U) != HAL_OK)
  {
    g_crsf_data.frame_error_counter++;
    g_restart_pending   = true;
    g_last_restart_tick = HAL_GetTick();
  }
}

void CRSF_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  (void)HAL_UART_AbortReceive(huart);
  if (HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U) != HAL_OK)
  {
    g_crsf_data.frame_error_counter++;
    g_restart_pending   = true;
    g_last_restart_tick = HAL_GetTick();
  }
}

static uint8_t crsf_crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;

  for (uint8_t i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0xD5U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

static void crsf_reset_parser(void)
{
  g_parser_state   = CRSF_STATE_WAIT_ADDRESS;
  g_expected_length = 0U;
  g_payload_index   = 0U;
}

static void crsf_handle_frame(const uint8_t *frame, uint8_t length)
{
  if (length < CRSF_PAYLOAD_MIN_LENGTH)
  {
    g_crsf_data.frame_error_counter++;
    return;
  }

  const uint8_t received_crc  = frame[length - 1U];
  const uint8_t computed_crc  = crsf_crc8(frame, length - 1U);

  if (received_crc != computed_crc)
  {
    g_crsf_data.frame_error_counter++;
    return;
  }

  const uint8_t frame_type    = frame[0U];
  const uint8_t payload_len   = length - 2U;
  const uint8_t *payload      = &frame[1U];

  switch (frame_type)
  {
    case CRSF_FRAME_TYPE_RC_CHANNELS:
      crsf_parse_rc_channels(payload, payload_len);
      break;

    default:
      /* 其他帧先不鸟它 */
      break;
  }
}

static void crsf_parse_rc_channels(const uint8_t *payload, uint8_t length)
{
  if (length < 22U)
  {
    g_crsf_data.frame_error_counter++;
    return;
  }

  uint32_t bit_buffer = 0U;
  uint8_t  bits_in_buffer = 0U;
  uint8_t  channel_index = 0U;

  for (uint8_t i = 0U; i < length; ++i)
  {
    bit_buffer |= ((uint32_t)payload[i] << bits_in_buffer);
    bits_in_buffer += 8U;

    while ((bits_in_buffer >= 11U) && (channel_index < CRSF_CHANNEL_COUNT))
    {
      g_crsf_data.channels[channel_index] = (uint16_t)(bit_buffer & 0x07FFU);
      bit_buffer >>= 11U;
      bits_in_buffer -= 11U;
      channel_index++;
    }
  }

  while (channel_index < CRSF_CHANNEL_COUNT)
  {
    g_crsf_data.channels[channel_index++] = 992U;
  }

  g_crsf_data.timestamp_ms = HAL_GetTick();
  g_crsf_data.link_active  = true;
  g_crsf_data.frame_counter++;
  g_last_frame_tick = g_crsf_data.timestamp_ms;
}
