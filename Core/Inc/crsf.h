/**
 ******************************************************************************
 * @file    crsf.h
 * @brief   CRSF协议解析模块接口
 * @version 1.0
 * @date    2025-01-22
 ******************************************************************************
 * @attention
 *
 * CRSF协议来自TBS Crossfire系统，具有高更新率特性
 * - 波特率：420000
 * - 帧类型：0x16 (RC Channels Packed)
 * - 通道：16路，11-bit
 *
 ******************************************************************************
 */

#ifndef CRSF_H
#define CRSF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define CRSF_CHANNEL_COUNT      16U
#define CRSF_TIMEOUT_MS         1000U
#define CRSF_CHANNEL_VALUE_MIN      172U
#define CRSF_CHANNEL_VALUE_NEUTRAL  992U
#define CRSF_CHANNEL_VALUE_MAX      1811U

typedef struct
{
  uint16_t channels[CRSF_CHANNEL_COUNT];
  uint32_t timestamp_ms;
  bool     link_active;
  uint32_t frame_counter;
  uint32_t frame_error_counter;
} CRSF_Data_t;

void CRSF_Init(void);
void CRSF_ProcessByte(uint8_t byte);
void CRSF_Update(void);
bool CRSF_IsLinkActive(void);
void CRSF_GetData(CRSF_Data_t *dest);
void CRSF_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void CRSF_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_H */
