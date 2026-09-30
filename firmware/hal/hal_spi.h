#ifndef HAL_SPI_H
#define HAL_SPI_H

#include "driverlib.h"
#include "device.h"

// ============================================================
// LAUNCHXL-F280049C + BOOSTXL-DRV8323RS
// 飞线适配后的最终 SPI 映射
//
// ENABLE -> GPIO13
// SCLK   -> GPIO56 / SPIA_CLK
// SDI    -> GPIO16 / SPIA_SIMO
// SDO    -> GPIO17 / SPIA_SOMI
// nSCS   -> GPIO57
// ============================================================

#define DRV8323_EN_GPIO        13U
#define DRV8323_CS_GPIO        57U

// DRV8323 register address
#define DRV8323_REG_FAULT1     0x00U
#define DRV8323_REG_FAULT2     0x01U
#define DRV8323_REG_DRIVER     0x02U
#define DRV8323_REG_GATE_HS    0x03U
#define DRV8323_REG_GATE_LS    0x04U
#define DRV8323_REG_OCP        0x05U
#define DRV8323_REG_CSA        0x06U

// ============================================================
// CCS Expressions debug variables
// ============================================================

extern volatile uint16_t drv_reg0_raw;
extern volatile uint16_t drv_reg1_raw;
extern volatile uint16_t drv_reg2_raw;
extern volatile uint16_t drv_reg3_raw;
extern volatile uint16_t drv_reg4_raw;
extern volatile uint16_t drv_reg5_raw;
extern volatile uint16_t drv_reg6_raw;

extern volatile uint16_t drv_reg0_readback;
extern volatile uint16_t drv_reg1_readback;
extern volatile uint16_t drv_reg2_readback;
extern volatile uint16_t drv_reg3_readback;
extern volatile uint16_t drv_reg4_readback;
extern volatile uint16_t drv_reg5_readback;
extern volatile uint16_t drv_reg6_readback;

extern volatile uint32_t drv_spi_test_counter;

// GPIO state debug
extern volatile uint16_t drv_enable_state;
extern volatile uint16_t drv_cs_state;

// ============================================================
// API
// ============================================================

void HAL_setupSPI(void);

uint16_t HAL_writeReadSPI(uint16_t txData);

uint16_t HAL_DRV8323_readRegister(uint16_t reg);

void HAL_DRV8323_writeRegister(uint16_t reg,
                               uint16_t data);

void HAL_DRV8323_readAllTestRegisters(void);

void HAL_configureDRV8323RS(void);

#endif
