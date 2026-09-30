/**
 * @file    hal_epwm.c
 * @brief   LAUNCHXL-F280049C + BOOSTXL-DRV8323RS
 *          30 kHz 三相中心对齐互补 PWM
 */

#include "hal_epwm.h"


// ============================================================
// PWM GPIO PinMux
// ============================================================

void HAL_setupPWMPins(void)
{
    // ========================================================
    // Phase U / A
    //
    // GPIO10 -> EPWM6A -> INHA
    // GPIO11 -> EPWM6B -> INLA
    // ========================================================

    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPinConfig(GPIO_11_EPWM6B);

    GPIO_setPadConfig(10U, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(11U, GPIO_PIN_TYPE_STD);


    // ========================================================
    // Phase V / B
    //
    // GPIO8 -> EPWM5A -> INHB
    // GPIO9 -> EPWM5B -> INLB
    // ========================================================

    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setPinConfig(GPIO_9_EPWM5B);

    GPIO_setPadConfig(8U, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(9U, GPIO_PIN_TYPE_STD);


    // ========================================================
    // Phase W / C
    //
    // GPIO4 -> EPWM3A -> INHC
    // GPIO5 -> EPWM3B -> INLC
    // ========================================================

    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    GPIO_setPadConfig(4U, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(5U, GPIO_PIN_TYPE_STD);
}


// ============================================================
// Configure one ePWM module
// ============================================================

static void initSinglePWMBase(uint32_t base)
{
    // ========================================================
    // 1. Time Base
    // ========================================================

    EPWM_setTimeBaseCounterMode(
        base,
        EPWM_COUNTER_MODE_UP_DOWN
    );

    EPWM_setTimeBasePeriod(
        base,
        PWM_PRD_TICKS
    );

    EPWM_setTimeBaseCounter(
        base,
        0U
    );

    EPWM_setPhaseShift(
        base,
        0U
    );

    // TBCLK = SYSCLK / 1 / 1
    EPWM_setClockPrescaler(
        base,
        EPWM_CLOCK_DIVIDER_1,
        EPWM_HSCLOCK_DIVIDER_1
    );


    // ========================================================
    // 2. 初始 50% duty
    // ========================================================

    EPWM_setCounterCompareValue(
        base,
        EPWM_COUNTER_COMPARE_A,
        PWM_PRD_TICKS / 2U
    );


    // ========================================================
    // 3. Action Qualifier
    //
    // Up-count CMPA:
    //      A -> LOW
    //
    // Down-count CMPA:
    //      A -> HIGH
    //
    // 中心对齐 PWM
    // ========================================================

    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA
    );

    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA
    );


    // ========================================================
    // 4. Dead Band
    //
    // EPWMxB 由 A 自动生成互补输出
    // ========================================================

    EPWM_setDeadBandDelayMode(
        base,
        EPWM_DB_RED,
        true
    );

    EPWM_setDeadBandDelayMode(
        base,
        EPWM_DB_FED,
        true
    );


    // RED 正极性
    EPWM_setDeadBandDelayPolarity(
        base,
        EPWM_DB_RED,
        EPWM_DB_POLARITY_ACTIVE_HIGH
    );

    // FED 反极性
    EPWM_setDeadBandDelayPolarity(
        base,
        EPWM_DB_FED,
        EPWM_DB_POLARITY_ACTIVE_LOW
    );


    // 100 ns
    EPWM_setRisingEdgeDelayCount(
        base,
        DEADTIME_TICKS
    );

    EPWM_setFallingEdgeDelayCount(
        base,
        DEADTIME_TICKS
    );
}


// ============================================================
// Setup three-phase PWM
// ============================================================

void HAL_setupPWMs(void)
{
    /*
     * 配置过程中先冻结全部 ePWM TBCLK，
     * 让三个 PWM 最终同时开始。
     */
    SysCtl_disablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC
    );


    // ========================================================
    // U / V / W
    // ========================================================

    initSinglePWMBase(PWM_U_BASE);
    initSinglePWMBase(PWM_V_BASE);
    initSinglePWMBase(PWM_W_BASE);


    /*
     * 当前阶段：
     *
     * 不配置 ADC SOCA
     * 不配置 PWM interrupt
     *
     * 单纯验证 PWM。
     */


    // ========================================================
    // 同时启动三个 PWM
    // ========================================================

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC
    );
}


// ============================================================
// Force PWM off
// ============================================================

void HAL_stopPWMs(void)
{
    /*
     * 软件强制 A/B 低电平。
     *
     * 后面正式工程会使用 Trip Zone 做硬件级关断，
     * 当前先提供简单软件停止接口。
     */

    EPWM_setActionQualifierContSWForceAction(
        PWM_U_BASE,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_OUTPUT_LOW
    );

    EPWM_setActionQualifierContSWForceAction(
        PWM_U_BASE,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_SW_OUTPUT_LOW
    );


    EPWM_setActionQualifierContSWForceAction(
        PWM_V_BASE,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_OUTPUT_LOW
    );

    EPWM_setActionQualifierContSWForceAction(
        PWM_V_BASE,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_SW_OUTPUT_LOW
    );


    EPWM_setActionQualifierContSWForceAction(
        PWM_W_BASE,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_OUTPUT_LOW
    );

    EPWM_setActionQualifierContSWForceAction(
        PWM_W_BASE,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_SW_OUTPUT_LOW
    );
}
