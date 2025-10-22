/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   独立看门狗(IWDG)管理实现
 ******************************************************************************
 * @attention
 *  本模块监控主循环运行状态，超时未喂狗时IWDG将触发系统复位
 ******************************************************************************
 */

#include "watchdog.h"
#include "main.h"

#define IWDG_TIMEOUT_MS        500U
#define IWDG_MAX_RELOAD        0x0FFFU
#define IWDG_PRESCALER_CODE    0x03U  /* 64分频 */
#define IWDG_PRESCALER_DIV     64U

static bool g_watchdog_reset = false;

static uint32_t watchdog_compute_reload(uint32_t timeout_ms)
{
  const uint32_t lsi_freq = 40000U; /* 估算值，足够覆盖LSI误差 */
  uint32_t ticks = (timeout_ms * (lsi_freq / IWDG_PRESCALER_DIV));
  ticks = (ticks + 999U) / 1000U;   /* 毫秒转tick，加一点余量 */
  if (ticks == 0U)
  {
    ticks = 1U;
  }
  if (ticks > IWDG_MAX_RELOAD)
  {
    ticks = IWDG_MAX_RELOAD;
  }
  return ticks;
}

bool Watchdog_WasReset(void)
{
  return g_watchdog_reset;
}

void Watchdog_Init(void)
{
#if defined(DEBUG)
  __HAL_DBGMCU_FREEZE_IWDG();
#endif

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    g_watchdog_reset = true;
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  __HAL_RCC_LSI_ENABLE();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  IWDG->KR  = 0x5555U;                              /* 解锁写保护 */
  IWDG->PR  = IWDG_PRESCALER_CODE;
  IWDG->RLR = watchdog_compute_reload(IWDG_TIMEOUT_MS);
  IWDG->KR  = 0xCCCCU;                              /* 启动看门狗 */
  IWDG->KR  = 0xAAAAU;                              /* 立即喂一次，避免误触发 */
}

void Watchdog_Kick(void)
{
  IWDG->KR = 0xAAAAU;
}
