/**
 * @file    hal_epwm.h
 * @brief   LAUNCHXL-F280049C + BOOSTXL-DRV8323RS
 *          三相互补 PWM Hardware Abstraction Layer
 */

#ifndef HAL_EPWM_H
#define HAL_EPWM_H

#include "driverlib.h"
#include "device.h"

// ============================================================
// PWM 参数
// ============================================================

#define SYSTEM_FREQ_HZ      100000000U
#define PWM_FREQ_HZ         30000U

/*
 * Up-Down center-aligned:
 *
 * Fpwm = TBCLK / (2 * TBPRD)
 *
 * TBPRD = 100 MHz / (2 * 30 kHz)
 *       ≈ 1666
 */
#define PWM_PRD_TICKS       \
    (SYSTEM_FREQ_HZ / (2U * PWM_FREQ_HZ))


// ============================================================
// Dead-time
// ============================================================

#define DEADTIME_NS         100U

/*
 * TBCLK = 100 MHz
 * 1 tick = 10 ns
 *
 * 100 ns = 10 ticks
 */
#define DEADTIME_TICKS      \
    ((DEADTIME_NS * 100U) / 1000U)


// ============================================================
// PWM physical mapping
//
// Phase U / A:
// GPIO10 -> EPWM6A -> INHA
// GPIO11 -> EPWM6B -> INLA
//
// Phase V / B:
// GPIO8  -> EPWM5A -> INHB
// GPIO9  -> EPWM5B -> INLB
//
// Phase W / C:
// GPIO4  -> EPWM3A -> INHC
// GPIO5  -> EPWM3B -> INLC
// ============================================================

#define PWM_U_BASE          EPWM6_BASE
#define PWM_V_BASE          EPWM5_BASE
#define PWM_W_BASE          EPWM3_BASE


// ============================================================
// Function API
// ============================================================

void HAL_setupPWMPins(void);

void HAL_setupPWMs(void);

void HAL_stopPWMs(void);


// ============================================================
// Set PWM compare values
// ============================================================

static inline void HAL_setPWMDutyCycles(uint16_t cmpU,
                                        uint16_t cmpV,
                                        uint16_t cmpW)
{
    // 防止非法比较值
    if(cmpU > PWM_PRD_TICKS)
    {
        cmpU = PWM_PRD_TICKS;
    }

    if(cmpV > PWM_PRD_TICKS)
    {
        cmpV = PWM_PRD_TICKS;
    }

    if(cmpW > PWM_PRD_TICKS)
    {
        cmpW = PWM_PRD_TICKS;
    }


    EPWM_setCounterCompareValue(
        PWM_U_BASE,
        EPWM_COUNTER_COMPARE_A,
        cmpU
    );

    EPWM_setCounterCompareValue(
        PWM_V_BASE,
        EPWM_COUNTER_COMPARE_A,
        cmpV
    );

    EPWM_setCounterCompareValue(
        PWM_W_BASE,
        EPWM_COUNTER_COMPARE_A,
        cmpW
    );
}


// ============================================================
// Duty 0.0 ~ 1.0
// ============================================================

static inline void HAL_setPWMDutyFloat(float dutyU,
                                       float dutyV,
                                       float dutyW)
{
    if(dutyU < 0.0f) dutyU = 0.0f;
    if(dutyU > 1.0f) dutyU = 1.0f;

    if(dutyV < 0.0f) dutyV = 0.0f;
    if(dutyV > 1.0f) dutyV = 1.0f;

    if(dutyW < 0.0f) dutyW = 0.0f;
    if(dutyW > 1.0f) dutyW = 1.0f;


    HAL_setPWMDutyCycles(
        (uint16_t)(dutyU * (float)PWM_PRD_TICKS),
        (uint16_t)(dutyV * (float)PWM_PRD_TICKS),
        (uint16_t)(dutyW * (float)PWM_PRD_TICKS)
    );
}

#endif
