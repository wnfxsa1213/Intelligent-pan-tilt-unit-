#include "safety_control.h"

#include <string.h>

#if SAFETY_DEBUG
#include <stdio.h>
#endif

#include "main.h"

#define SAFETY_CH5_INDEX           4U
#define SAFETY_CH8_INDEX           7U

#define SWITCH_THRESHOLD_LOW       800U
#define SWITCH_THRESHOLD_HIGH      1200U

typedef enum
{
  SWITCH_STATE_LOW  = 0U,
  SWITCH_STATE_HIGH = 1U
} SwitchState_t;

typedef struct
{
  SafetyMode_t  safety_mode;
  LaserState_t  laser_state;
  SwitchState_t ch5_state;
  SwitchState_t ch8_state;
  uint32_t      mode_change_count;
  bool          link_active;
} SafetyControl_Context_t;

static SafetyControl_Context_t g_ctx;

static void        safety_apply_laser_state(LaserState_t state);
static SwitchState_t safety_channel_to_switch(uint16_t value, SwitchState_t previous);

void SafetyControl_Init(void)
{
  memset(&g_ctx, 0, sizeof(g_ctx));
  g_ctx.safety_mode = SAFETY_MODE_LOCKED;
  g_ctx.laser_state = LASER_STATE_OFF;
  g_ctx.ch5_state   = SWITCH_STATE_LOW;
  g_ctx.ch8_state   = SWITCH_STATE_LOW;
  safety_apply_laser_state(LASER_STATE_OFF);
}

void SafetyControl_Update(const CRSF_Data_t *rc_data, bool has_new_frame)
{
  g_ctx.link_active    = CRSF_IsLinkActive();

  if ((rc_data != NULL) && has_new_frame)
  {
    const uint16_t ch5_value = rc_data->channels[SAFETY_CH5_INDEX];
    const uint16_t ch8_value = rc_data->channels[SAFETY_CH8_INDEX];

    g_ctx.ch5_state = safety_channel_to_switch(ch5_value, g_ctx.ch5_state);
    g_ctx.ch8_state = safety_channel_to_switch(ch8_value, g_ctx.ch8_state);
  }

  SafetyMode_t desired_mode = (g_ctx.ch5_state == SWITCH_STATE_HIGH) ?
                              SAFETY_MODE_UNLOCKED : SAFETY_MODE_LOCKED;
  LaserState_t desired_laser = (g_ctx.ch8_state == SWITCH_STATE_HIGH) ?
                               LASER_STATE_ON : LASER_STATE_OFF;

  if (!g_ctx.link_active)
  {
    desired_mode  = SAFETY_MODE_LOCKED;
    desired_laser = LASER_STATE_OFF;
  }

  if (desired_mode == SAFETY_MODE_LOCKED)
  {
    desired_laser = LASER_STATE_OFF;
  }

  const bool mode_changed  = (g_ctx.safety_mode != desired_mode);
  const bool laser_changed = (g_ctx.laser_state != desired_laser);

  if (mode_changed)
  {
    g_ctx.safety_mode = desired_mode;
    g_ctx.mode_change_count++;
  }

  if (laser_changed)
  {
    g_ctx.laser_state = desired_laser;
    safety_apply_laser_state(desired_laser);
  }
#if SAFETY_DEBUG
  if (mode_changed || laser_changed)
  {
    char debug_buf[128];
    const int len = snprintf(
        debug_buf,
        sizeof(debug_buf),
        "[SAFETY] mode=%u laser=%u link=%u count=%lu\r\n",
        (unsigned int)g_ctx.safety_mode,
        (unsigned int)g_ctx.laser_state,
        g_ctx.link_active ? 1U : 0U,
        (unsigned long)g_ctx.mode_change_count);
    if ((len > 0) && (len < (int)sizeof(debug_buf)))
    {
      UartSendText(debug_buf);
    }
  }
#endif
}

SafetyMode_t SafetyControl_GetMode(void)
{
  return g_ctx.safety_mode;
}

LaserState_t SafetyControl_GetLaserState(void)
{
  return g_ctx.laser_state;
}

bool SafetyControl_IsGimbalLocked(void)
{
  return (g_ctx.safety_mode == SAFETY_MODE_LOCKED);
}

static void safety_apply_laser_state(LaserState_t state)
{
  const GPIO_PinState gpio_state = (state == LASER_STATE_ON) ?
                                   GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(JIGUANG_GPIO_Port, JIGUANG_Pin, gpio_state);
}

static SwitchState_t safety_channel_to_switch(uint16_t value, SwitchState_t previous)
{
  if (value <= SWITCH_THRESHOLD_LOW)
  {
    return SWITCH_STATE_LOW;
  }

  if (value >= SWITCH_THRESHOLD_HIGH)
  {
    return SWITCH_STATE_HIGH;
  }

  return previous;
}
