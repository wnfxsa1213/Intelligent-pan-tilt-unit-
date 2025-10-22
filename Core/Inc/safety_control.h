/**
 * ****************************************************************************
 * @file    safety_control.h
 * @brief   云台安全控制模块 - 锁定逻辑与激光控制
 * @version 1.0
 * @date    2025-10-22
 * ****************************************************************************
 * @attention
 *
 * 本模块处理 CH5 保险拨杆锁定逻辑与 CH8 激光控制：
 * - 锁定状态优先级最高，强制舵机回中并关闭激光
 * - 支持通道值滞环防抖（800-1200 阈值）
 * - CRSF 失联自动锁定并关闭激光
 *
 * ****************************************************************************
 */
#ifndef SAFETY_CONTROL_H
#define SAFETY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef struct CRSF_Data CRSF_Data_t;

#ifndef SAFETY_DEBUG
#define SAFETY_DEBUG 0U
#endif

typedef enum
{
  SAFETY_MODE_LOCKED   = 0U,
  SAFETY_MODE_UNLOCKED = 1U
} SafetyMode_t;

typedef enum
{
  LASER_STATE_OFF = 0U,
  LASER_STATE_ON  = 1U
} LaserState_t;

/**
 * @brief  初始化安全控制模块
 * @details
 *         设置初始状态为锁定模式并关闭激光输出。
 *         要求在 GPIO 和 CRSF 模块初始化完成后调用一次。
 */
void SafetyControl_Init(void);

/**
 * @brief  更新安全控制状态
 * @details
 *         基于最新的 CRSF 遥控数据刷新锁定与激光状态。
 *         - CH5：<800 锁定、>1200 解锁、中间保持
 *         - CH8：<800 关闭激光、>1200 打开，中间保持
 *         - 失联或锁定状态会强制关闭激光
 *
 * @param  rc_data       指向 CRSF 数据快照，可为 NULL（无新帧）
 * @param  has_new_frame 指示是否收到最新帧，用于消抖与状态缓存
 */
void SafetyControl_Update(const CRSF_Data_t *rc_data, bool has_new_frame);

/**
 * @brief  获取当前保险模式
 * @return 当前安全模式（锁定/解锁）
 */
SafetyMode_t SafetyControl_GetMode(void);

/**
 * @brief  查询激光当前状态
 * @return 激光开关状态（打开/关闭）
 */
LaserState_t SafetyControl_GetLaserState(void);

/**
 * @brief  判断云台是否处于锁定态
 * @retval true  锁定中，需要回中舵机并禁止外部控制
 * @retval false 已解锁，可正常接受控制输入
 */
bool SafetyControl_IsGimbalLocked(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_CONTROL_H */
