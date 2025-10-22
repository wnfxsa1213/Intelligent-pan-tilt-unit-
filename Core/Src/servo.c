/**
 ******************************************************************************
 * @file    servo.c
 * @brief   数字舵机控制驱动实现
 * @version 1.0
 * @date    2025-01-22
 ******************************************************************************
 * @attention
 *
 * 本驱动实现角度到TIM5 PWM脉宽的转换，有效角度范围：0° ~ 270°
 * TIM基准：72MHz / 72 -> 1MHz，Period=3002 -> 333Hz
 *
 ******************************************************************************
 */

#include "servo.h"

#include "tim.h"  /* TIM5 句柄 */

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

/**
 * @brief  将舵机角度转换为PWM脉宽
 * @param  angle_deg  舵机角度 (度)，有效范围 [0, 270]
 * @retval PWM脉宽 (微秒)，范围 [500, 2500]
 * @note   超出范围的角度会被自动限制到边界值
 */
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

  if ((angle_deg < SERVO_MIN_ANGLE_DEG) || (angle_deg > SERVO_MAX_ANGLE_DEG))
  {
    return SERVO_STATUS_INVALID_PARAM;
  }

  if (g_servo_initialized == 0U)
  {
    return SERVO_STATUS_NOT_READY;
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
 * @brief  舵机周期性更新任务
 * @note   当前版本暂未实现，预留接口用于后续扩展平滑插值控制
 * @todo   V1.1版本计划实现角度平滑过渡算法
 */
void Servo_Update(void)
{
  /* TODO(laowang): 等正式开干插值/限速再填坑，先别乱用这接口 */
}
