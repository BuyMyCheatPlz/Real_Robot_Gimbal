#ifndef PID_PARAMETER_H
#define PID_PARAMETER_H

#include "gimbal_control.h"

uint8_t PidParameter_Parse(const char *command,
                           PidParameterUpdate_t *update);

#endif
