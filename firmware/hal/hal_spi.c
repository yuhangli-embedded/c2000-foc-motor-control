#include "hal_spi.h"

// ============================================================
// DRV8323RS SPI frame
//
// bit15    : R/W
//            0 = Write
//            1 = Read
//
// bit14:11 : Address
// bit10:0  : Data
// ============================================================

#define DRV8323_REG_WRITE(reg, data) \
    ((uint16_t)((((reg) & 0x0FU) << 11) | \
                ((data) & 0x07FFU)))

#define DRV8323_REG_READ(reg) \
    ((uint16_t)(0x8000U | \
                (((reg) & 0x0FU) << 11)))


// ============================================================
// Debug variables
// ============================================================

volatile uint16_t drv_reg0_raw = 0U;
volatile uint16_t drv_reg1_raw = 0U;
volatile uint16_t drv_reg2_raw = 0U;
volatile uint16_t drv_reg3_raw = 0U;
volatile uint16_t drv_reg4_raw = 0U;
volatile uint16_t drv_reg5_raw = 0U;
volatile uint16_t drv_reg6_raw = 0U;

volatile uint16_t drv_reg0_readback = 0U;
volatile uint16_t drv_reg1_readback = 0U;
volatile uint16_t drv_reg2_readback = 0U;
volatile uint16_t drv_reg3_readback = 0U;
volatile uint16_t drv_reg4_readback = 0U;
volatile uint16_t drv_reg5_readback = 0U;
volatile uint16_t drv_reg6_readback = 0U;

volatile uint32_t drv_spi_test_counter = 0U;

volatile uint16_t drv_enable_state = 0U;
volatile uint16_t drv_cs_state = 0U;


// ============================================================
// HAL_setupSPI
// ============================================================

void HAL_setupSPI(void)
{
    // ========================================================
    // 1. ENABLE -> GPIO13
    // ========================================================

    GPIO_setPinConfig(GPIO_13_GPIO13);

    GPIO_setPadConfig(DRV8323_EN_GPIO,
                      GPIO_PIN_TYPE_STD);

    GPIO_setDirectionMode(DRV8323_EN_GPIO,
                          GPIO_DIR_MODE_OUT);

    // startup: keep disabled
    GPIO_writePin(DRV8323_EN_GPIO, 0U);


    // ========================================================
    // 2. SPIA SIMO
    // GPIO16 -> DRV SDI
    // ========================================================

    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);

    GPIO_setPadConfig(16U,
                      GPIO_PIN_TYPE_STD);

    GPIO_setQualificationMode(16U,
                              GPIO_QUAL_ASYNC);


    // ========================================================
    // 3. SPIA SOMI
    // GPIO17 <- DRV SDO
    // ========================================================

    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);

    // SDO is open-drain
    GPIO_setPadConfig(17U,
                      GPIO_PIN_TYPE_PULLUP);

    GPIO_setQualificationMode(17U,
                              GPIO_QUAL_ASYNC);


    // ========================================================
    // 4. SPIA CLK
    // GPIO56 -> DRV SCLK
    // ========================================================

    GPIO_setPinConfig(GPIO_56_SPIA_CLK);

    GPIO_setPadConfig(56U,
                      GPIO_PIN_TYPE_STD);

    GPIO_setQualificationMode(56U,
                              GPIO_QUAL_ASYNC);


    // ========================================================
    // 5. Software nSCS
    // GPIO57 -> DRV nSCS
    // ========================================================

    GPIO_setPinConfig(GPIO_57_GPIO57);

    GPIO_setPadConfig(DRV8323_CS_GPIO,
                      GPIO_PIN_TYPE_PULLUP);

    GPIO_setDirectionMode(DRV8323_CS_GPIO,
                          GPIO_DIR_MODE_OUT);

    // nSCS idle HIGH
    GPIO_writePin(DRV8323_CS_GPIO, 1U);


    // ========================================================
    // 6. SPIA peripheral
    // ========================================================

    SPI_disableModule(SPIA_BASE);

    /*
     * 先用低速 500 kHz 排障。
     *
     * DRV8323:
     * SCLK idle LOW
     * SDI sampled on falling edge
     *
     * 因此使用 POL0PHA1。
     */
    SPI_setConfig(SPIA_BASE,
                  DEVICE_LSPCLK_FREQ,
                  SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER,
                  500000U,
                  16U);

    SPI_disableFIFO(SPIA_BASE);

    SPI_disableLoopback(SPIA_BASE);

    SPI_enableModule(SPIA_BASE);


    // ========================================================
    // 7. Enable DRV8323
    // ========================================================

    DEVICE_DELAY_US(100U);

    GPIO_writePin(DRV8323_EN_GPIO, 1U);

    // Give the driver generous wake-up time
    DEVICE_DELAY_US(10000U);


    // GPIO status for CCS
    drv_enable_state =
        GPIO_readPin(DRV8323_EN_GPIO);

    drv_cs_state =
        GPIO_readPin(DRV8323_CS_GPIO);
}


// ============================================================
// 16-bit SPI transaction
// ============================================================

uint16_t HAL_writeReadSPI(uint16_t txData)
{
    uint16_t rxData;

    // nSCS LOW
    GPIO_writePin(DRV8323_CS_GPIO, 0U);

    DEVICE_DELAY_US(2U);

    // 16-bit TX
    SPI_writeDataBlockingNonFIFO(SPIA_BASE,
                                 txData);

    // 16-bit RX
    rxData =
        SPI_readDataBlockingNonFIFO(SPIA_BASE);

    DEVICE_DELAY_US(2U);

    // nSCS HIGH
    GPIO_writePin(DRV8323_CS_GPIO, 1U);

    DEVICE_DELAY_US(5U);

    return rxData;
}


// ============================================================
// Read one register
// ============================================================

uint16_t HAL_DRV8323_readRegister(uint16_t reg)
{
    uint16_t rawData;

    rawData =
        HAL_writeReadSPI(
            DRV8323_REG_READ(reg)
        );

    return (rawData & 0x07FFU);
}


// ============================================================
// Write one register
// ============================================================

void HAL_DRV8323_writeRegister(uint16_t reg,
                               uint16_t data)
{
    (void)HAL_writeReadSPI(
        DRV8323_REG_WRITE(reg, data)
    );
}


// ============================================================
// Read registers 0~6
// ============================================================

void HAL_DRV8323_readAllTestRegisters(void)
{
    drv_reg0_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(0U)
        );

    drv_reg0_readback =
        drv_reg0_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg1_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(1U)
        );

    drv_reg1_readback =
        drv_reg1_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg2_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(2U)
        );

    drv_reg2_readback =
        drv_reg2_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg3_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(3U)
        );

    drv_reg3_readback =
        drv_reg3_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg4_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(4U)
        );

    drv_reg4_readback =
        drv_reg4_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg5_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(5U)
        );

    drv_reg5_readback =
        drv_reg5_raw & 0x07FFU;

    DEVICE_DELAY_US(20U);


    drv_reg6_raw =
        HAL_writeReadSPI(
            DRV8323_REG_READ(6U)
        );

    drv_reg6_readback =
        drv_reg6_raw & 0x07FFU;


    drv_enable_state =
        GPIO_readPin(DRV8323_EN_GPIO);

    drv_cs_state =
        GPIO_readPin(DRV8323_CS_GPIO);

    drv_spi_test_counter++;
}


// ============================================================
// DRV8323 startup
// ============================================================

void HAL_configureDRV8323RS(void)
{
    DEVICE_DELAY_US(10000U);

    HAL_DRV8323_readAllTestRegisters();
}
