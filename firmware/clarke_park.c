#include "clarke_park.h"


// ============================================================
// Constants
// ============================================================

#define TWO_THIRDS_F           0.66666666666666666667f
#define HALF_F                 0.5f

#define SQRT3_OVER_2_F         0.86602540378443864676f


// ============================================================
// Clarke Transform
// ============================================================

void MC_clarke(
    float ia,
    float ib,
    float ic,
    float *alpha,
    float *beta
)
{
    *alpha =
        TWO_THIRDS_F *
        (
            ia
            - HALF_F * ib
            - HALF_F * ic
        );


    *beta =
        TWO_THIRDS_F *
        SQRT3_OVER_2_F *
        (
            ib - ic
        );
}


// ============================================================
// Park Transform
// ============================================================

void MC_park(
    float alpha,
    float beta,
    float sin_theta,
    float cos_theta,
    float *d,
    float *q
)
{
    *d =
        alpha * cos_theta +
        beta  * sin_theta;


    *q =
       -alpha * sin_theta +
        beta  * cos_theta;
}


// ============================================================
// Inverse Park Transform
// ============================================================

void MC_invPark(
    float d,
    float q,
    float sin_theta,
    float cos_theta,
    float *alpha,
    float *beta
)
{
    *alpha =
        d * cos_theta -
        q * sin_theta;


    *beta =
        d * sin_theta +
        q * cos_theta;
}
