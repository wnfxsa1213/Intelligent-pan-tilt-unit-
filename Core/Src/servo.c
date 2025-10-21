/**
 ******************************************************************************
 * @file    servo.c
 * @brief   数字舵机控制驱动实现
 * @version 1.0
 * @date    2025-01-22
 ******************************************************************************
 * @attention
 *
 * 这个SB驱动把角度换算成TIM5 PWM脉宽，别tm乱传负角度。
 * TIM基准：72MHz / 72 -> 1MHz，Period=3002 -> 333Hz
 *
 ******************************************************************************
 */

#include "servo.h"

#include "tim.h"  /* TIM5 句柄 */

#define SERVO_CHANNEL_COUNT      2U
#define SERVO_MIN_ANGLE_DEG      0.0f
#define SERVO_MAX_ANGLE_DEG      270.0f
#define SERVO_CENTER_ANGLE_DEG   135.0f

#define SERVO_MIN_PULSE_US       500U
#define SERVO_MAX_PULSE_US       2500U
#define SERVO_RANGE_PULSE_US     (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
#define SERVO_ROUNDING_OFFSET    0.5f

typedef struct
{
  float    angle_deg;
  uint16_t pulse_us;
} ServoChannel_t;

static ServoChannel_t g_servo_channels[SERVO_CHANNEL_COUNT];
static uint8_t        g_servo_initialized = 0U;

/* 这个SB工具函数把角度映射到PWM脉宽，别越界 */
static uint16_t servo_angle_to_pulse(float angle_deg)
{
  if (angle_deg <= SERVO_MIN_ANGLE_DEG)
  {
    return SERVO_MIN_PULSE_US;
  }
  if (angle_deg >= SERVO_MAX_ANGLE_DEG)
  {
    return SERVO_MAX_PULSE_US;
  }

  const float normalized = (angle_deg - SERVO_MIN_ANGLE_DEG) /
                           (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
  const float pulse      = (float)SERVO_MIN_PULSE_US + normalized * (float)SERVO_RANGE_PULSE_US;
  return (uint16_t)(pulse + SERVO_ROUNDING_OFFSET);
}

static TIM_HandleTypeDef *servo_get_tim(ServoID_t id)
{
  (void)id;
  return &htim5;
}

static uint32_t servo_get_channel(ServoID_t id)
{
  if (id >= SERVO_CHANNEL_COUNT)
  {
    return TIM_CHANNEL_1;
  }

  return (id == SERVO_PITCH) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static ServoStatus_t servo_apply_output(ServoID_t id, uint16_t pulse)
{
  TIM_HandleTypeDef *handle = servo_get_tim(id);
  const uint32_t channel    = servo_get_channel(id);

  __HAL_TIM_SET_COMPARE(handle, channel, pulse);
  return SERVO_STATUS_OK;
}

/**
 * @brief  舵机模块初始化
 * @retval HAL_OK 初始化成功
 * @retval HAL_ERROR PWM启动失败
 */
HAL_StatusTypeDef Servo_Init(void)
{
  if (g_servo_initialized != 0U)
  {
    return HAL_OK;
  }

  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    return HAL_ERROR;
  }

  const uint16_t center_pulse = servo_angle_to_pulse(SERVO_CENTER_ANGLE_DEG);

  if ((servo_apply_output(SERVO_PITCH, center_pulse) != SERVO_STATUS_OK) ||
      (servo_apply_output(SERVO_YAW, center_pulse) != SERVO_STATUS_OK))
  {
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    return HAL_ERROR;
  }

  g_servo_channels[SERVO_PITCH].angle_deg = SERVO_CENTER_ANGLE_DEG;
  g_servo_channels[SERVO_PITCH].pulse_us  = center_pulse;
  g_servo_channels[SERVO_YAW].angle_deg   = SERVO_CENTER_ANGLE_DEG;
  g_servo_channels[SERVO_YAW].pulse_us    = center_pulse;

  g_servo_initialized = 1U;

  return HAL_OK;
}

/**
 * @brief  设置舵机目标角度
 * @param  id         舵机通道
 * @param  angle_deg  角度（单位：度，0~270）
 * @retval ServoStatus_t 操作状态
 */
ServoStatus_t Servo_SetAngle(ServoID_t id, float angle_deg)
{
  if (id >= SERVO_CHANNEL_COUNT)
  {
    return SERVO_STATUS_INVALID_PARAM;
  }

  if ((angle_deg < -360.0f) || (angle_deg > 360.0f))
  {
    return SERVO_STATUS_INVALID_PARAM;
  }

  if (g_servo_initialized == 0U)
  {
    return SERVO_STATUS_NOT_READY;
  }

  if (angle_deg < SERVO_MIN_ANGLE_DEG)
  {
    angle_deg = SERVO_MIN_ANGLE_DEG;
  }
  else if (angle_deg > SERVO_MAX_ANGLE_DEG)
  {
    angle_deg = SERVO_MAX_ANGLE_DEG;
  }

  const uint16_t pulse = servo_angle_to_pulse(angle_deg);
  if (servo_apply_output(id, pulse) != SERVO_STATUS_OK)
  {
    return SERVO_STATUS_DRIVER_ERROR;
  }

  g_servo_channels[id].angle_deg = angle_deg;
  g_servo_channels[id].pulse_us  = pulse;

  return SERVO_STATUS_OK;
}

/**
 * @brief  获取当前舵机角度
 * @param  id 舵机通道
 * @retval 当前角度，未初始化时返回中位
 */
float Servo_GetAngle(ServoID_t id)
{
  if (id >= SERVO_CHANNEL_COUNT)
  {
    return SERVO_CENTER_ANGLE_DEG;
  }

  if (g_servo_initialized == 0U)
  {
    return SERVO_CENTER_ANGLE_DEG;
  }

  return g_servo_channels[id].angle_deg;
}

/**
 * @brief  周期性任务占位，后续加插值或保护逻辑
 */
void Servo_Update(void)
{
  /* 这破函数后面搞平滑控制的时候再说，现在留空 */
}
