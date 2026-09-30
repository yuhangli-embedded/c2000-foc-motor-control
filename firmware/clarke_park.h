#ifndef CLARKE_PARK_H
#define CLARKE_PARK_H


#ifdef __cplusplus
extern "C" {
#endif


// ============================================================
// Clarke transform
//
// abc -> alpha/beta
//
// Uses amplitude-invariant Clarke:
//
// alpha = 2/3 * (Ia - 0.5*Ib - 0.5*Ic)
//
// beta  = 2/3 * (sqrt(3)/2) * (Ib - Ic)
//
// For balanced three-phase currents:
//
// Ia + Ib + Ic = 0
//
// then:
//
// alpha = Ia
// ============================================================

void MC_clarke(
    float ia,
    float ib,
    float ic,
    float *alpha,
    float *beta
);


// ============================================================
// Park transform
//
// alpha/beta -> d/q
//
// d =  alpha*cos(theta) + beta*sin(theta)
// q = -alpha*sin(theta) + beta*cos(theta)
//
// theta = electrical rotor angle
// ============================================================

void MC_park(
    float alpha,
    float beta,
    float sin_theta,
    float cos_theta,
    float *d,
    float *q
);


// ============================================================
// Inverse Park transform
//
// d/q -> alpha/beta
//
// alpha = d*cos(theta) - q*sin(theta)
// beta  = d*sin(theta) + q*cos(theta)
// ============================================================

void MC_invPark(
    float d,
    float q,
    float sin_theta,
    float cos_theta,
    float *alpha,
    float *beta
);


#ifdef __cplusplus
}
#endif


#endif
