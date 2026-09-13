#ifndef PITCH_APPROACH_H
#define PITCH_APPROACH_H

typedef struct
{
    float static_voltage;
} PitchApproachState_t;

void PitchApproach_Reset(PitchApproachState_t *state);
float PitchApproach_LimitSpeed(float speed_target_deg_s,
                               float error_deg,
                               float measurement_deg);
float PitchApproach_UpdateStaticErrorComp(PitchApproachState_t *state,
                                          float error_deg,
                                          float measurement_deg,
                                          float speed_deg_s,
                                          float control_to_motor_sign,
                                          float dt_s);

#endif
