#include "pid_parameter.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    const char *name;
    PidParameterId_t id;
} PidParameterName_t;

static const PidParameterName_t parameter_names[] = {
    {"PITCH_KP_POS", PID_PARAM_PITCH_KP_POS},
    {"PITCH_KI_POS", PID_PARAM_PITCH_KI_POS},
    {"PITCH_KD_POS", PID_PARAM_PITCH_KD_POS},
    {"PITCH_KP_SPD", PID_PARAM_PITCH_KP_SPD},
    {"PITCH_KI_SPD", PID_PARAM_PITCH_KI_SPD},
    {"PITCH_KD_SPD", PID_PARAM_PITCH_KD_SPD},
    {"YAW_KP_POS", PID_PARAM_YAW_KP_POS},
    {"YAW_KI_POS", PID_PARAM_YAW_KI_POS},
    {"YAW_KD_POS", PID_PARAM_YAW_KD_POS},
    {"YAW_KP_SPD", PID_PARAM_YAW_KP_SPD},
    {"YAW_KI_SPD", PID_PARAM_YAW_KI_SPD},
    {"YAW_KD_SPD", PID_PARAM_YAW_KD_SPD},
    {"PITCH_GRAVITY_FF", PID_PARAM_PITCH_GRAVITY_FF},
    {"PITCH_GRAVITY_FF_MAX_VOLTAGE", PID_PARAM_PITCH_GRAVITY_FF}
};

uint8_t PidParameter_Parse(const char *command,
                           PidParameterUpdate_t *update)
{
    const char *equals;
    char *end;
    size_t key_length;
    size_t index;
    uint8_t found = 0U;

    if ((command == 0) || (update == 0)) return 0U;
    equals = strchr(command, '=');
    if (equals == 0) return 0U;
    key_length = (size_t)(equals - command);
    for (index = 0U;
         index < (sizeof(parameter_names) / sizeof(parameter_names[0]));
         ++index)
    {
        if ((strlen(parameter_names[index].name) == key_length) &&
            (strncmp(command, parameter_names[index].name, key_length) == 0))
        {
            update->id = parameter_names[index].id;
            found = 1U;
            break;
        }
    }
    if (found == 0U) return 0U;

    update->value = strtof(equals + 1, &end);
    if (end == (equals + 1)) return 0U;
    while ((*end == ' ') || (*end == '\t')) ++end;
    if ((*end != '\0') || (isfinite(update->value) == 0) ||
        (update->value < 0.0f) || (update->value > ONLINE_PID_VALUE_MAX))
        return 0U;
    return 1U;
}
