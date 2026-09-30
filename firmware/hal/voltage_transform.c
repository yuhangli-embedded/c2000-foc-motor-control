#include "voltage_transform.h"


#define HALF_F                 0.5f
#define SQRT3_OVER_2_F         0.86602540378443864676f


// ============================================================
// Internal clamp
// ============================================================

static float clamp01(float x)
{
    if(x > 1.0f)
    {
        return 1.0f;
    }

    if(x < 0.0f)
    {
        return 0.0f;
    }

    return x;
}


// ============================================================
// Inverse Clarke
// ============================================================

void MC_invClarke(
    float alpha,
    float beta,
    float *u,
    float *v,
    float *w
)
{
    *u =
        alpha;


    *v =
       -HALF_F * alpha +
        SQRT3_OVER_2_F * beta;


    *w =
       -HALF_F * alpha -
        SQRT3_OVER_2_F * beta;
}


// ============================================================
// Normalized phase voltage -> duty
// ============================================================

void MC_phaseVoltageToDuty(
    float u,
    float v,
    float w,
    float *duty_u,
    float *duty_v,
    float *duty_w
)
{
    *duty_u =
        clamp01(
            0.5f +
            0.5f * u
        );


    *duty_v =
        clamp01(
            0.5f +
            0.5f * v
        );


    *duty_w =
        clamp01(
            0.5f +
            0.5f * w
        );
}
