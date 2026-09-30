#include "hal_adc.h"

#include "driverlib.h"
#include "device.h"


// ============================================================
// Current ADC setup
//
// U = ADCINA9
// V = ADCINC0
// W = ADCINB2
//
// Sampling is synchronized by EPWM6 SOCA.
// ============================================================

void HAL_ADC_setupCurrentSense(void)
{
    // ========================================================
    // ADC clocks
    // ========================================================

    ADC_setPrescaler(
        ADCA_BASE,
        ADC_CLK_DIV_4_0
    );


    ADC_setPrescaler(
        ADCB_BASE,
        ADC_CLK_DIV_4_0
    );


    ADC_setPrescaler(
        ADCC_BASE,
        ADC_CLK_DIV_4_0
    );


    // ========================================================
    // Internal 3.3 V references
    // ========================================================

    ADC_setVREF(
        ADCA_BASE,
        ADC_REFERENCE_INTERNAL,
        ADC_REFERENCE_3_3V
    );


    ADC_setVREF(
        ADCB_BASE,
        ADC_REFERENCE_INTERNAL,
        ADC_REFERENCE_3_3V
    );


    ADC_setVREF(
        ADCC_BASE,
        ADC_REFERENCE_INTERNAL,
        ADC_REFERENCE_3_3V
    );


    // ========================================================
    // Enable converters
    // ========================================================

    ADC_enableConverter(
        ADCA_BASE
    );


    ADC_enableConverter(
        ADCB_BASE
    );


    ADC_enableConverter(
        ADCC_BASE
    );


    DEVICE_DELAY_US(
        1000U
    );


    // ========================================================
    // U phase
    //
    // BOOSTXL ISENA -> ADCINA9
    // ========================================================

    ADC_setupSOC(
        ADCA_BASE,
        ADC_SOC_NUMBER0,
        ADC_TRIGGER_EPWM6_SOCA,
        ADC_CH_ADCIN9,
        30U
    );


    // ========================================================
    // V phase
    //
    // BOOSTXL ISENB -> ADCINC0
    // ========================================================

    ADC_setupSOC(
        ADCC_BASE,
        ADC_SOC_NUMBER0,
        ADC_TRIGGER_EPWM6_SOCA,
        ADC_CH_ADCIN0,
        30U
    );


    // ========================================================
    // W phase
    //
    // BOOSTXL ISENC -> ADCINB2
    // ========================================================

    ADC_setupSOC(
        ADCB_BASE,
        ADC_SOC_NUMBER0,
        ADC_TRIGGER_EPWM6_SOCA,
        ADC_CH_ADCIN2,
        30U
    );


    // ========================================================
    // ADCA EOC0 -> ADCINT1
    //
    // ISR reads all three ADC result registers.
    // ========================================================

    ADC_setInterruptSource(
        ADCA_BASE,
        ADC_INT_NUMBER1,
        ADC_SOC_NUMBER0
    );


    ADC_clearInterruptStatus(
        ADCA_BASE,
        ADC_INT_NUMBER1
    );


    ADC_enableInterrupt(
        ADCA_BASE,
        ADC_INT_NUMBER1
    );
}


// ============================================================
// EPWM6 SOCA trigger
//
// Verified sampling point:
// TBCTR = PERIOD
// ============================================================

void HAL_ADC_setupPWMTrigger(void)
{
    EPWM_setADCTriggerSource(
        EPWM6_BASE,
        EPWM_SOC_A,
        EPWM_SOC_TBCTR_PERIOD
    );


    EPWM_setADCTriggerEventPrescale(
        EPWM6_BASE,
        EPWM_SOC_A,
        1U
    );


    EPWM_enableADCTrigger(
        EPWM6_BASE,
        EPWM_SOC_A
    );
}


// ============================================================
// Raw samples
// ============================================================

uint16_t HAL_ADC_readCurrentU(void)
{
    return
        ADC_readResult(
            ADCARESULT_BASE,
            ADC_SOC_NUMBER0
        );
}


uint16_t HAL_ADC_readCurrentV(void)
{
    return
        ADC_readResult(
            ADCCRESULT_BASE,
            ADC_SOC_NUMBER0
        );
}


uint16_t HAL_ADC_readCurrentW(void)
{
    return
        ADC_readResult(
            ADCBRESULT_BASE,
            ADC_SOC_NUMBER0
        );
}


// ============================================================
// Interrupt clear
// ============================================================

void HAL_ADC_clearCurrentInterrupt(void)
{
    ADC_clearInterruptStatus(
        ADCA_BASE,
        ADC_INT_NUMBER1
    );


    Interrupt_clearACKGroup(
        INTERRUPT_ACK_GROUP1
    );
}
