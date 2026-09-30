#ifndef VOLTAGE_TRANSFORM_H
#define VOLTAGE_TRANSFORM_H


#ifdef __cplusplus
extern "C" {
#endif


// ============================================================
// Inverse Clarke
//
// alpha/beta -> U/V/W
//
// U = alpha
//
// V = -0.5*alpha + sqrt(3)/2*beta
//
// W = -0.5*alpha - sqrt(3)/2*beta
// ============================================================

void MC_invClarke(
    float alpha,
    float beta,
    float *u,
    float *v,
    float *w
);


// ============================================================
// Three-phase normalized voltage -> PWM duty
//
// Input phase command range:
//
// -1.0 ... +1.0
//
// Mapping:
//
// duty = 0.5 + 0.5 * command
//
// Therefore:
//
// command = 0     -> duty = 0.5
// command = +1    -> duty = 1.0
// command = -1    -> duty = 0.0
//
// Output duty is clamped to 0...1.
// ============================================================

void MC_phaseVoltageToDuty(
    float u,
    float v,
    float w,
    float *duty_u,
    float *duty_v,
    float *duty_w
);


#ifdef __cplusplus
}
#endif


#endif
