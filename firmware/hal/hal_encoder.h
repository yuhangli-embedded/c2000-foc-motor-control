#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include "driverlib.h"
#include "device.h"

// ============================================================
// MT6701 ABZ -> F280049C eQEP2
//
// A -> GPIO14 / EQEP2A
// B -> GPIO15 / EQEP2B
// Z -> GPIO26 / EQEP2I
// ============================================================

#define ENCODER_EQEP_BASE      EQEP2_BASE

void HAL_setupEncoderPins(void);
void HAL_setupEncoder(void);

#endif
