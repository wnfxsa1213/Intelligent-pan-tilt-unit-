#ifndef SAFETY_CONTROL_H
#define SAFETY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "crsf.h"

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

void         SafetyControl_Init(void);
void         SafetyControl_Update(const CRSF_Data_t *rc_data, bool has_new_frame);
SafetyMode_t SafetyControl_GetMode(void);
LaserState_t SafetyControl_GetLaserState(void);
bool         SafetyControl_IsGimbalLocked(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_CONTROL_H */
