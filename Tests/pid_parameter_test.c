#include "pid_parameter.h"
#include <assert.h>
#include <math.h>

typedef struct
{
    const char *command;
    PidParameterId_t expected_id;
} ParseCase_t;

int main(void)
{
    const ParseCase_t cases[] = {
        {"PITCH_KP_POS=1", PID_PARAM_PITCH_KP_POS},
        {"PITCH_KI_POS=2", PID_PARAM_PITCH_KI_POS},
        {"PITCH_KD_POS=3", PID_PARAM_PITCH_KD_POS},
        {"PITCH_KP_SPD=4", PID_PARAM_PITCH_KP_SPD},
        {"PITCH_KI_SPD=5", PID_PARAM_PITCH_KI_SPD},
        {"PITCH_KD_SPD=6", PID_PARAM_PITCH_KD_SPD},
        {"YAW_KP_POS=7", PID_PARAM_YAW_KP_POS},
        {"YAW_KI_POS=8", PID_PARAM_YAW_KI_POS},
        {"YAW_KD_POS=9", PID_PARAM_YAW_KD_POS},
        {"YAW_KP_SPD=10", PID_PARAM_YAW_KP_SPD},
        {"YAW_KI_SPD=11", PID_PARAM_YAW_KI_SPD},
        {"YAW_KD_SPD=12", PID_PARAM_YAW_KD_SPD}
    };
    PidParameterUpdate_t update;
    unsigned int index;

    for (index = 0U; index < (sizeof(cases) / sizeof(cases[0])); ++index)
    {
        assert(PidParameter_Parse(cases[index].command, &update) != 0U);
        assert(update.id == cases[index].expected_id);
        assert(fabsf(update.value - (float)(index + 1U)) < 0.0001f);
    }

    assert(PidParameter_Parse("PITCH_GRAVITY_FF=1500", &update) != 0U);
    assert(update.id == PID_PARAM_PITCH_GRAVITY_FF);
    assert(fabsf(update.value - 1500.0f) < 0.0001f);
    assert(PidParameter_Parse("PITCH_GRAVITY_FF=30000", &update) != 0U);
    assert(update.id == PID_PARAM_PITCH_GRAVITY_FF);
    assert(fabsf(update.value - 30000.0f) < 0.0001f);
    assert(PidParameter_Parse("PITCH_GRAVITY_FF_MAX_VOLTAGE=30000",
                             &update) != 0U);
    assert(update.id == PID_PARAM_PITCH_GRAVITY_FF);
    assert(fabsf(update.value - 30000.0f) < 0.0001f);

    /* 拒绝含义不明确的旧命令，避免同时修改两个轴。 */
    assert(PidParameter_Parse("KP_POS=1", &update) == 0U);
    assert(PidParameter_Parse("KP_SPD=1", &update) == 0U);
    assert(PidParameter_Parse("YAW_KP_SPD=-1", &update) == 0U);
    assert(PidParameter_Parse("YAW_KP_SPD=nan", &update) == 0U);
    assert(PidParameter_Parse("YAW_KP_SPD=2junk", &update) == 0U);
    return 0;
}
