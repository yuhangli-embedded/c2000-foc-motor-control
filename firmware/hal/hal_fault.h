#ifndef HAL_FAULT_H
#define HAL_FAULT_H

#include <stdint.h>


// ============================================================
// Gate-driver ENABLE
//
// GPIO13 -> BOOSTXL ENABLE
// ============================================================

void HAL_Fault_setupGateEnable(void);

void HAL_Fault_gateEnable(void);

void HAL_Fault_gateDisable(void);


// ============================================================
// DRV8323 nFAULT
//
// BOOSTXL nFAULT -> GPIO58
// LaunchPad Pin34
// ============================================================

void HAL_Fault_setupNFAULT(void);

uint16_t HAL_Fault_readNFAULT(void);


#endif
