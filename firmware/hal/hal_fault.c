#include "hal_fault.h"

#include "driverlib.h"


#define DRV_ENABLE_GPIO       13U
#define DRV_NFAULT_GPIO       58U


// ============================================================
// Gate ENABLE
// ============================================================

void HAL_Fault_setupGateEnable(void)
{
    GPIO_setPinConfig(
        GPIO_13_GPIO13
    );


    GPIO_setPadConfig(
        DRV_ENABLE_GPIO,
        GPIO_PIN_TYPE_STD
    );


    GPIO_setDirectionMode(
        DRV_ENABLE_GPIO,
        GPIO_DIR_MODE_OUT
    );


    GPIO_writePin(
        DRV_ENABLE_GPIO,
        0U
    );
}


void HAL_Fault_gateEnable(void)
{
    GPIO_writePin(
        DRV_ENABLE_GPIO,
        1U
    );
}


void HAL_Fault_gateDisable(void)
{
    GPIO_writePin(
        DRV_ENABLE_GPIO,
        0U
    );
}


// ============================================================
// DRV8323 nFAULT
// ============================================================

void HAL_Fault_setupNFAULT(void)
{
    GPIO_setPinConfig(
        GPIO_58_GPIO58
    );


    GPIO_setPadConfig(
        DRV_NFAULT_GPIO,
        GPIO_PIN_TYPE_PULLUP
    );


    GPIO_setDirectionMode(
        DRV_NFAULT_GPIO,
        GPIO_DIR_MODE_IN
    );


    GPIO_setQualificationMode(
        DRV_NFAULT_GPIO,
        GPIO_QUAL_ASYNC
    );
}


uint16_t HAL_Fault_readNFAULT(void)
{
    return
        GPIO_readPin(
            DRV_NFAULT_GPIO
        );
}
