/**
 ******************************************************************************
 * @file    servo.h
 * @brief   数字舵机控制驱动 - PWM接口封装
 * @version 1.0
 * @date    2025-01-22
 ******************************************************************************
 * @attention
 *
 * 本模块基于TIM5 CH1/CH2输出333Hz PWM信号，用于控制双轴数字舵机
 * - 角度范围：0° ~ 270°
 * - 脉宽范围：500µs ~ 2500µs
 * - 默认上电中位：135°
 *
 ******************************************************************************
 */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define SERVO_CHANNEL_COUNT      2U
#define SERVO_MIN_ANGLE_DEG      0.0f
#define SERVO_MAX_ANGLE_DEG      270.0f
#define SERVO_CENTER_ANGLE_DEG   135.0f

/**
 * @brief 舵机通道枚举
 */
typedef enum
{
  SERVO_PITCH = 0U,  /**< TIM5_CH1 - 俯仰轴 */
  SERVO_YAW   = 1U   /**< TIM5_CH2 - 水平轴 */
} ServoID_t;

/**
 * @brief 舵机操作状态
 */
typedef enum
{
  SERVO_STATUS_OK = 0,
  SERVO_STATUS_NOT_READY,
  SERVO_STATUS_INVALID_PARAM,
  SERVO_STATUS_DRIVER_ERROR
} ServoStatus_t;

HAL_StatusTypeDef Servo_Init(void);
ServoStatus_t     Servo_SetAngle(ServoID_t id, float angle_deg);
float             Servo_GetAngle(ServoID_t id);
void              Servo_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
