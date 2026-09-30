#ifndef MOTOR_APP_H
#define MOTOR_APP_H


#include <stdint.h>


// ============================================================
// Motor state
// ============================================================

typedef enum
{
    MOTOR_STATE_INIT = 0,

    MOTOR_STATE_CURRENT_CAL,

    MOTOR_STATE_ALIGN,

    MOTOR_STATE_READY,

    MOTOR_STATE_FAULT_TEST_READY,

    MOTOR_STATE_RUN_PREP,

    MOTOR_STATE_RUN,

    MOTOR_STATE_STOP,

    MOTOR_STATE_FAULT,

    MOTOR_STATE_RECOVERY_PREP

} MotorState;


// ============================================================
// Motor command
// ============================================================

typedef enum
{
    MOTOR_CMD_NONE = 0,

    MOTOR_CMD_START,

    MOTOR_CMD_STOP,

    MOTOR_CMD_RESET,

    MOTOR_CMD_FAULT_TEST

} MotorCommand;


// ============================================================
// Motor fault
// ============================================================

typedef enum
{
    MOTOR_FAULT_NONE = 0,

    MOTOR_FAULT_OVERCURRENT,

    MOTOR_FAULT_DRV8323,

    MOTOR_FAULT_ENCODER,

    MOTOR_FAULT_SOFTWARE_TEST

} MotorFault;


// ============================================================
// Hardware / control callbacks
//
// motor_app does NOT know GPIO / SPI / PWM details.
// ============================================================

typedef struct
{
    void (*setNeutralPWM)(void);

    void (*gateEnable)(void);

    void (*gateDisable)(void);

    void (*configureDRV)(void);

    void (*startCurrentCalibration)(void);

    void (*startAlignment)(void);

    void (*captureAlignmentOffset)(void);

    void (*resetRunController)(void);

} MotorAppCallbacks;


// ============================================================
// External status supplied by main/control layer
// ============================================================

typedef struct
{
    uint16_t current_calibrated;

    uint16_t drv_nfault_level;

    uint16_t stop_confirm_counter;

} MotorAppInputs;


// ============================================================
// Main application state
// ============================================================

typedef struct
{
    volatile MotorState state;

    volatile MotorCommand command;

    volatile MotorFault fault_code;


    volatile uint32_t state_time_ms;


    volatile uint16_t current_loop_active;

    volatile uint16_t speed_loop_active;


    volatile uint16_t software_fault_inject;


    volatile uint16_t drv_configured;

    volatile uint16_t alignment_capture_done;


    volatile uint16_t drv_nfault_at_fault;


    MotorAppCallbacks callbacks;

} MotorApp;


// ============================================================
// Init
// ============================================================

void MotorApp_init(
    MotorApp *app,
    const MotorAppCallbacks *callbacks
);


// ============================================================
// 1 ms state-machine step
// ============================================================

void MotorApp_step(
    MotorApp *app,
    const MotorAppInputs *inputs
);


// ============================================================
// Fault entry
//
// May be called from ADC ISR.
// ============================================================

void MotorApp_enterFault(
    MotorApp *app,
    MotorFault fault
);


// ============================================================
// Advance application time by 1 ms
// ============================================================

void MotorApp_tick1ms(
    MotorApp *app
);


#endif
