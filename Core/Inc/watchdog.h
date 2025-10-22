/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   独立看门狗(IWDG)管理接口
 ******************************************************************************
 * @attention
 *  本模块提供系统保护机制，定期喂狗以防止IWDG触发复位
 ******************************************************************************
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

void Watchdog_Init(void);
void Watchdog_Kick(void);
bool Watchdog_WasReset(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
