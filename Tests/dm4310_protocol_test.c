#include "dm4310.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void assert_close(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    DM4310_t motor;
    DM4310_t motor_id5;
    DM4310_t slot_motor;
    DM4310_t invalid_motor;
    uint8_t command[8];
    uint8_t snapshot[8];
    uint8_t feedback[8] = {
        0x1FU, 0xFFU,       /* 8191 个计数 */
        0xCFU, 0xC7U,       /* -12345 -> -123.45 rpm */
        0xFCU, 0x18U,       /* -1000 mA */
        55U, 60U            /* 绕组 / PCB 温度 */
    };
    uint16_t std_id = 0U;
    uint8_t id;
    uint8_t byte_index;
    uint8_t slot;
    int32_t accumulated_command = 0;
    uint16_t sample;

    DM4310_Init(&motor, 1U, 100.0f, 0.0f);
    memset(command, 0xAA, sizeof(command));
    assert(DM4310_PackCurrentCommand(&motor, 0x1234, &std_id,
                                     command) != 0U);
    assert(std_id == DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert((command[0] == 0xE8U) && (command[1] == 0x03U));
    assert((command[2] == 0xAAU) && (command[7] == 0xAAU));

    memset(command, 0, sizeof(command));
    assert(DM4310_PackCurrentCommand(&motor, 20000, &std_id,
                                     command) != 0U);
    assert((command[0] == 0xE8U) && (command[1] == 0x03U));
    assert(DM4310_PackCurrentCommand(&motor, -20000, &std_id,
                                     command) != 0U);
    assert((command[0] == 0x18U) && (command[1] == 0xFCU));

    DM4310_Init(&motor_id5, 5U, 0.0f, 0.0f);
    memset(command, 0, sizeof(command));
    assert(DM4310_PackCurrentCommand(&motor_id5, -1, &std_id,
                                     command) != 0U);
    assert(std_id == DM4310_CURRENT_CONTROL_ID_5_TO_8);
    assert((command[0] == 0xFFU) && (command[1] == 0xFFU));

    for (id = DM4310_MOTOR_ID_MIN; id <= DM4310_MOTOR_ID_MAX; ++id)
    {
        DM4310_Init(&slot_motor, id, 0.0f, 0.0f);
        memset(command, 0, sizeof(command));
        assert(DM4310_PackCurrentCommand(&slot_motor, 3,
                                         &std_id, command) != 0U);
        assert(std_id == ((id <= 4U) ?
               DM4310_CURRENT_CONTROL_ID_1_TO_4 :
               DM4310_CURRENT_CONTROL_ID_5_TO_8));
        slot = (uint8_t)((id - 1U) % 4U);
        for (byte_index = 0U; byte_index < sizeof(command); ++byte_index)
        {
            if (byte_index == (uint8_t)(slot * 2U))
                assert(command[byte_index] == 0x03U);
            else if (byte_index == (uint8_t)(slot * 2U + 1U))
                assert(command[byte_index] == 0x00U);
            else
                assert(command[byte_index] == 0U);
        }
    }

    DM4310_Decode(&motor, feedback, 1234U);
    assert(motor.encoder == 8191U);
    assert_close(motor.single_turn_position_rad,
                 8191.0f * 6.2831853071795864769f / 8192.0f, 0.00001f);
    assert_close(motor.speed_rpm, -123.45f, 0.0001f);
    assert(motor.torque_current_ma == -1000);
    assert(motor.winding_temperature == 55U);
    assert(motor.pcb_temperature == 60U);
    assert((motor.online != 0U) && (motor.last_update_ms == 1234U));

    DM4310_SetSpeed(&motor, 1000.0f);
    assert(DM4310_Update(&motor, 0.001f) == 1000);

    DM4310_Init(&motor, 1U, 0.0f, 0.0f);
    DM4310_Decode(&motor, feedback, 1234U);
    DM4310_SetCurrentFeedforward(&motor, 1000);
    assert(DM4310_Update(&motor, 0.001f) == 1000);
    DM4310_SetCurrentFeedforward(&motor, 20000);
    assert(DM4310_Update(&motor, 0.001f) == 1000);

    /* 小力矩 PID 和前馈值必须通过有界脉冲密度平均保留到整数协议格式中，
     * 不能直接截断成 0 并造成较大的位置死区。 */
    DM4310_Init(&motor, 1U, 0.0f, 0.0f);
    motor.online = 1U;
    DM4310_SetCurrentFeedforward(&motor, 0.25f);
    for (sample = 0U; sample < 400U; ++sample)
    {
        int16_t command_value = DM4310_Update(&motor, 0.001f);
        assert((command_value >= 0) && (command_value <= 1));
        accumulated_command += command_value;
    }
    assert(accumulated_command == 100);

    DM4310_Init(&invalid_motor, 0U, 0.0f, 0.0f);
    memset(command, 0x5A, sizeof(command));
    memcpy(snapshot, command, sizeof(snapshot));
    assert(DM4310_PackCurrentCommand(&invalid_motor, 1, &std_id,
                                     command) == 0U);
    assert(memcmp(command, snapshot, sizeof(command)) == 0);
    return 0;
}
