#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>


// ============================================================
// Current ADC hardware configuration
//
// U -> ADCA / ADCINA9
// V -> ADCC / ADCINC0
// W -> ADCB / ADCINB2
//
// Trigger:
// EPWM6 SOCA at TBCTR = PERIOD
// ============================================================

void HAL_ADC_setupCurrentSense(void);


// ============================================================
// EPWM6 synchronized ADC trigger
// ============================================================

void HAL_ADC_setupPWMTrigger(void);


// ============================================================
// Raw phase-current ADC samples
// ============================================================

uint16_t HAL_ADC_readCurrentU(void);

uint16_t HAL_ADC_readCurrentV(void);

uint16_t HAL_ADC_readCurrentW(void);


// ============================================================
// ADC interrupt helpers
// ============================================================

void HAL_ADC_clearCurrentInterrupt(void);


#endif
