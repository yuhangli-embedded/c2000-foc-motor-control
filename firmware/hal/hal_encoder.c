#include "hal_encoder.h"

void HAL_setupEncoderPins(void)
{
    // --------------------------------------------------------
    // MT6701 A -> GPIO14 -> EQEP2A
    // --------------------------------------------------------
    GPIO_setPinConfig(GPIO_14_EQEP2A);
    GPIO_setPadConfig(14U, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14U, GPIO_QUAL_SYNC);

    // --------------------------------------------------------
    // MT6701 B -> GPIO15 -> EQEP2B
    // --------------------------------------------------------
    GPIO_setPinConfig(GPIO_15_EQEP2B);
    GPIO_setPadConfig(15U, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(15U, GPIO_QUAL_SYNC);

    // --------------------------------------------------------
    // MT6701 Z -> GPIO26 -> EQEP2I
    // --------------------------------------------------------
    GPIO_setPinConfig(GPIO_26_EQEP2I);
    GPIO_setPadConfig(26U, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26U, GPIO_QUAL_SYNC);
}


void HAL_setupEncoder(void)
{
    // Disable module while configuring
    EQEP_disableModule(ENCODER_EQEP_BASE);

    // --------------------------------------------------------
    // Quadrature mode
    //
    // 2X resolution:
    // both A/B edge information used for position tracking
    // --------------------------------------------------------
    EQEP_setDecoderConfig(
        ENCODER_EQEP_BASE,
        EQEP_CONFIG_2X_RESOLUTION |
        EQEP_CONFIG_QUADRATURE |
        EQEP_CONFIG_NO_SWAP
    );

    // --------------------------------------------------------
    // Position counter
    //
    // For bring-up we let it free-run over full 32-bit range.
    // Do NOT reset on index yet.
    // --------------------------------------------------------
    EQEP_setPositionCounterConfig(
        ENCODER_EQEP_BASE,
        EQEP_POSITION_RESET_MAX_POS,
        0xFFFFFFFFUL
    );

    EQEP_setPosition(
        ENCODER_EQEP_BASE,
        0U
    );

    // --------------------------------------------------------
    // Latch position on index rising edge
    // --------------------------------------------------------
    EQEP_setLatchMode(
        ENCODER_EQEP_BASE,
        EQEP_LATCH_UNIT_TIME_OUT |
        EQEP_LATCH_RISING_INDEX
    );

    // ============================================================
    // Unit Timer
    //
    // SYSCLK = 100 MHz
    // 10 ms = 1,000,000 SYSCLK cycles
    // ============================================================

    EQEP_enableUnitTimer(
        ENCODER_EQEP_BASE,
        1000000UL
    );

    // --------------------------------------------------------
    // Enable eQEP
    // --------------------------------------------------------
    EQEP_enableModule(
        ENCODER_EQEP_BASE
    );
}
