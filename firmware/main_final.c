#include "driverlib.h"
#include "device.h"

#include "hal_spi.h"
#include "hal_epwm.h"
#include "hal_encoder.h"
#include "hal_adc.h"
#include "hal_fault.h"

#include "motor_control.h"
#include "motor_app.h"


// ============================================================
// Motor / Encoder
// ============================================================

#define MOTOR_POLE_PAIRS                 7
#define ENCODER_CPR                      4096

#define PI_F                             3.14159265358979323846f


// ============================================================
// Current sensing
// ============================================================

#define CURRENT_A_PER_COUNT              0.005756f
#define CURRENT_CAL_SAMPLES              1000U


// ============================================================
// Alignment
// ============================================================

#define ALIGN_DUTY_U                     0.53f
#define ALIGN_DUTY_V                     0.47f
#define ALIGN_DUTY_W                     0.47f


// ============================================================
// Current PI
// ============================================================

#define ID_PI_KP                         0.15f
#define ID_PI_KI                         0.00050f

#define IQ_PI_KP                         0.15f
#define IQ_PI_KI                         0.00050f


// ============================================================
// Voltage limits
// ============================================================

#define VD_LIMIT                         0.010f
#define VQ_LIMIT                         0.100f


// ============================================================
// Speed PI
// ============================================================

#define SPEED_PI_KP                      0.00040f
#define SPEED_PI_KI                      0.000010f

#define SPEED_IQ_LIMIT_A                 0.120f

#define SPEED_TO_IQ_SIGN                 (-1.0f)

#define SPEED_COMMAND_LIMIT_RPM          150.0f


// ============================================================
// Speed estimator
// ============================================================

#define SPEED_RPM_PER_COUNT              1.46484375f
#define SPEED_FILTER_ALPHA               0.20f


// ============================================================
// Controlled STOP
// ============================================================

#define STOP_SPEED_THRESHOLD_RPM         3.0f
#define STOP_CONFIRM_COUNT               20U


// ============================================================
// Current protection
// ============================================================

#define CURRENT_TRIP_A                   0.300f
#define CURRENT_TRIP_CONFIRM_COUNT       3U


// ============================================================
// Core modules
// ============================================================

static MotorControl motor_control;

static MotorApp motor_app;


// ============================================================
// CCS operating variables
// ============================================================

volatile MotorCommand motor_command =
    MOTOR_CMD_NONE;


volatile uint16_t software_fault_inject =
    0U;


volatile float speed_command_rpm =
    100.0f;


// ============================================================
// CCS status mirrors
// ============================================================

volatile MotorState motor_state =
    MOTOR_STATE_INIT;


volatile MotorFault motor_fault_code =
    MOTOR_FAULT_NONE;


volatile float speed_rpm_filtered =
    0.0f;


// ============================================================
// DRV8323 fault diagnostics
// ============================================================

volatile uint16_t drv_nfault_level =
    1U;


volatile uint16_t drv_nfault_at_fault =
    1U;


volatile uint16_t drv_fault_status1_latched =
    0U;


volatile uint16_t drv_fault_status2_latched =
    0U;


// ============================================================
// Current calibration
// ============================================================

volatile uint16_t current_calibrated =
    0U;


volatile uint16_t current_rawU =
    0U;


volatile uint16_t current_rawV =
    0U;


volatile uint16_t current_rawW =
    0U;


volatile float current_offsetU =
    0.0f;


volatile float current_offsetV =
    0.0f;


volatile float current_offsetW =
    0.0f;


// ============================================================
// Phase-current feedback
// ============================================================

volatile float current_A_U =
    0.0f;


volatile float current_A_V =
    0.0f;


volatile float current_A_W =
    0.0f;


// ============================================================
// dq current diagnostics
// ============================================================

volatile float current_alpha =
    0.0f;


volatile float current_beta =
    0.0f;


volatile float current_d =
    0.0f;


volatile float current_q =
    0.0f;


// ============================================================
// Electrical angle
// ============================================================

volatile float alignment_mech_deg =
    0.0f;


volatile float electrical_offset_deg =
    0.0f;


volatile float electrical_angle_deg =
    0.0f;


volatile float electrical_angle_rad =
    0.0f;


// ============================================================
// Current references / controller diagnostics
// ============================================================

volatile float id_ref_A =
    0.0f;


volatile float iq_ref_A =
    0.0f;


volatile float id_error_A =
    0.0f;


volatile float iq_error_A =
    0.0f;


volatile float vd_command =
    0.0f;


volatile float vq_command =
    0.0f;


// ============================================================
// Speed estimator
// ============================================================

volatile int32_t speed_position =
    0;


volatile int32_t speed_position_prev =
    0;


volatile int32_t speed_delta_count =
    0;


volatile float speed_rpm_raw =
    0.0f;


// ============================================================
// Speed controller
// ============================================================

volatile float speed_ref_rpm =
    0.0f;


volatile float speed_error_rpm =
    0.0f;


volatile float speed_torque_command_A =
    0.0f;


// ============================================================
// Current calibration internals
// ============================================================

static volatile uint16_t cal_active =
    0U;


static volatile uint16_t cal_count =
    0U;


static volatile uint32_t cal_sumU =
    0U;


static volatile uint32_t cal_sumV =
    0U;


static volatile uint32_t cal_sumW =
    0U;


// ============================================================
// Current protection
// ============================================================

static volatile uint16_t current_trip_counter =
    0U;


// ============================================================
// Speed estimator state
// ============================================================

static volatile uint16_t speed_estimator_initialized =
    0U;


// ============================================================
// Controlled STOP
// ============================================================

static volatile uint16_t stop_confirm_counter =
    0U;


// ============================================================
// Utility
// ============================================================

static float wrap360(
    float angle
)
{
    while(angle >= 360.0f)
    {
        angle -= 360.0f;
    }


    while(angle < 0.0f)
    {
        angle += 360.0f;
    }


    return angle;
}


static float absFloat(
    float x
)
{
    if(x < 0.0f)
    {
        return -x;
    }


    return x;
}


static float clampFloat(
    float x,
    float min_value,
    float max_value
)
{
    if(x > max_value)
    {
        return max_value;
    }


    if(x < min_value)
    {
        return min_value;
    }


    return x;
}


// ============================================================
// Mechanical angle
// ============================================================

static float readMechanicalAngleDeg(void)
{
    int32_t position;

    int32_t position_mod;


    position =
        (int32_t)EQEP_getPosition(
            ENCODER_EQEP_BASE
        );


    position_mod =
        position %
        ENCODER_CPR;


    if(position_mod < 0)
    {
        position_mod +=
            ENCODER_CPR;
    }


    return
        (
            (float)position_mod *
            360.0f
        )
        /
        (float)ENCODER_CPR;
}


// ============================================================
// Neutral PWM
// ============================================================

static void setNeutralPWM(void)
{
    HAL_setPWMDutyFloat(
        0.50f,
        0.50f,
        0.50f
    );
}


// ============================================================
// Current calibration
// ============================================================

static void startCurrentCalibration(void)
{
    current_calibrated =
        0U;


    cal_active =
        0U;


    cal_count =
        0U;


    cal_sumU =
        0U;


    cal_sumV =
        0U;


    cal_sumW =
        0U;


    cal_active =
        1U;
}


// ============================================================
// Alignment
// ============================================================

static void startAlignment(void)
{
    HAL_setPWMDutyFloat(
        ALIGN_DUTY_U,
        ALIGN_DUTY_V,
        ALIGN_DUTY_W
    );
}


// ============================================================
// Capture electrical offset
//
// Experimentally verified:
//
// theta_e = -7 * theta_m + offset
//
// During alignment:
// theta_e = 0
//
// therefore:
//
// offset = +7 * theta_m
// ============================================================

static void captureAlignmentOffset(void)
{
    alignment_mech_deg =
        readMechanicalAngleDeg();


    electrical_offset_deg =
        wrap360(
            (float)MOTOR_POLE_PAIRS *
            alignment_mech_deg
        );
}


// ============================================================
// Reset closed-loop states
// ============================================================

static void resetRunController(void)
{
    MotorControl_reset(
        &motor_control
    );


    id_ref_A =
        0.0f;


    iq_ref_A =
        0.0f;


    id_error_A =
        0.0f;


    iq_error_A =
        0.0f;


    vd_command =
        0.0f;


    vq_command =
        0.0f;


    speed_ref_rpm =
        0.0f;


    speed_error_rpm =
        0.0f;


    speed_torque_command_A =
        0.0f;


    speed_position =
        0;


    speed_position_prev =
        0;


    speed_delta_count =
        0;


    speed_rpm_raw =
        0.0f;


    speed_rpm_filtered =
        0.0f;


    speed_estimator_initialized =
        0U;


    stop_confirm_counter =
        0U;


    current_trip_counter =
        0U;
}


// ============================================================
// MotorApp callbacks
// ============================================================

static void appConfigureDRV(void)
{
    HAL_configureDRV8323RS();
}


static void appResetRunController(void)
{
    resetRunController();
}


// ============================================================
// Speed estimator + speed controller
//
// Executed at eQEP Unit Timeout (~10 ms).
// ============================================================

static void updateSpeedLoop(void)
{
    float command_rpm;


    speed_position =
        (int32_t)EQEP_getPositionLatch(
            ENCODER_EQEP_BASE
        );


    // ========================================================
    // Initialize estimator
    // ========================================================

    if(speed_estimator_initialized == 0U)
    {
        speed_position_prev =
            speed_position;


        speed_delta_count =
            0;


        speed_rpm_raw =
            0.0f;


        speed_rpm_filtered =
            0.0f;


        speed_estimator_initialized =
            1U;


        return;
    }


    // ========================================================
    // Delta position
    // ========================================================

    speed_delta_count =
        speed_position -
        speed_position_prev;


    speed_position_prev =
        speed_position;


    // ========================================================
    // Raw RPM
    // ========================================================

    speed_rpm_raw =
        (float)speed_delta_count *
        SPEED_RPM_PER_COUNT;


    // ========================================================
    // LPF
    // ========================================================

    speed_rpm_filtered +=
        SPEED_FILTER_ALPHA *
        (
            speed_rpm_raw -
            speed_rpm_filtered
        );


    if(absFloat(speed_rpm_filtered) < 0.001f)
    {
        speed_rpm_filtered =
            0.0f;
    }


    // ========================================================
    // RUN
    // ========================================================

    if(
        (motor_app.state == MOTOR_STATE_RUN)
        &&
        (motor_app.speed_loop_active != 0U)
    )
    {
        command_rpm =
            clampFloat(
                speed_command_rpm,
                -SPEED_COMMAND_LIMIT_RPM,
                SPEED_COMMAND_LIMIT_RPM
            );


        speed_ref_rpm =
            command_rpm;


        MotorControl_runSpeedLoop(
            &motor_control,

            speed_ref_rpm,
            speed_rpm_filtered,

            (float *)&speed_error_rpm,
            (float *)&speed_torque_command_A,
            (float *)&iq_ref_A
        );
    }


    // ========================================================
    // Controlled STOP
    // ========================================================

    else if(
        (motor_app.state == MOTOR_STATE_STOP)
        &&
        (motor_app.speed_loop_active != 0U)
    )
    {
        speed_ref_rpm =
            0.0f;


        MotorControl_runSpeedLoop(
            &motor_control,

            0.0f,
            speed_rpm_filtered,

            (float *)&speed_error_rpm,
            (float *)&speed_torque_command_A,
            (float *)&iq_ref_A
        );


        if(
            absFloat(speed_rpm_filtered) <=
            STOP_SPEED_THRESHOLD_RPM
        )
        {
            if(
                stop_confirm_counter <
                STOP_CONFIRM_COUNT
            )
            {
                stop_confirm_counter++;
            }
        }
        else
        {
            stop_confirm_counter =
                0U;
        }
    }
}


// ============================================================
// 30 kHz ADC / current-control ISR
// ============================================================

__interrupt void adcA1ISR(void)
{
    float zeroU;
    float zeroV;
    float zeroW;

    float mechanical_angle_deg;

    float duty_u;
    float duty_v;
    float duty_w;

    float abs_u;
    float abs_v;
    float abs_w;

    float peak_current;


    // ========================================================
    // ADC acquisition
    // ========================================================

    current_rawU =
        HAL_ADC_readCurrentU();


    current_rawV =
        HAL_ADC_readCurrentV();


    current_rawW =
        HAL_ADC_readCurrentW();


    // ========================================================
    // Current offset calibration
    // ========================================================

    if(cal_active != 0U)
    {
        cal_sumU +=
            current_rawU;


        cal_sumV +=
            current_rawV;


        cal_sumW +=
            current_rawW;


        cal_count++;


        if(
            cal_count >=
            CURRENT_CAL_SAMPLES
        )
        {
            current_offsetU =
                (float)cal_sumU /
                (float)CURRENT_CAL_SAMPLES;


            current_offsetV =
                (float)cal_sumV /
                (float)CURRENT_CAL_SAMPLES;


            current_offsetW =
                (float)cal_sumW /
                (float)CURRENT_CAL_SAMPLES;


            cal_active =
                0U;


            current_calibrated =
                1U;
        }
    }


    if(current_calibrated != 0U)
    {
        // ====================================================
        // ADC counts -> current
        // ====================================================

        zeroU =
            (float)current_rawU -
            current_offsetU;


        zeroV =
            (float)current_rawV -
            current_offsetV;


        zeroW =
            (float)current_rawW -
            current_offsetW;


        current_A_U =
            zeroU *
            CURRENT_A_PER_COUNT;


        current_A_V =
            zeroV *
            CURRENT_A_PER_COUNT;


        current_A_W =
            zeroW *
            CURRENT_A_PER_COUNT;


        // ====================================================
        // Electrical angle
        // ====================================================

        mechanical_angle_deg =
            readMechanicalAngleDeg();


        electrical_angle_deg =
            MotorControl_getElectricalAngleDeg(
                &motor_control,
                mechanical_angle_deg,
                electrical_offset_deg
            );


        electrical_angle_rad =
            electrical_angle_deg *
            PI_F /
            180.0f;


        // ====================================================
        // Software phase-current protection
        // ====================================================

        if(motor_app.current_loop_active != 0U)
        {
            abs_u =
                absFloat(
                    current_A_U
                );


            abs_v =
                absFloat(
                    current_A_V
                );


            abs_w =
                absFloat(
                    current_A_W
                );


            peak_current =
                abs_u;


            if(abs_v > peak_current)
            {
                peak_current =
                    abs_v;
            }


            if(abs_w > peak_current)
            {
                peak_current =
                    abs_w;
            }


            if(peak_current > CURRENT_TRIP_A)
            {
                if(
                    current_trip_counter <
                    CURRENT_TRIP_CONFIRM_COUNT
                )
                {
                    current_trip_counter++;
                }
            }
            else
            {
                current_trip_counter =
                    0U;
            }


            if(
                current_trip_counter >=
                CURRENT_TRIP_CONFIRM_COUNT
            )
            {
                MotorApp_enterFault(
                    &motor_app,
                    MOTOR_FAULT_OVERCURRENT
                );
            }
        }


        // ====================================================
        // FOC current loop
        // ====================================================

        if(
            (motor_app.current_loop_active != 0U)
            &&
            (motor_app.state != MOTOR_STATE_FAULT)
        )
        {
            MotorControl_runCurrentLoop(
                &motor_control,

                current_A_U,
                current_A_V,
                current_A_W,

                electrical_angle_rad,

                id_ref_A,
                iq_ref_A,

                (float *)&current_alpha,
                (float *)&current_beta,

                (float *)&current_d,
                (float *)&current_q,

                (float *)&id_error_A,
                (float *)&iq_error_A,

                (float *)&vd_command,
                (float *)&vq_command
            );


            MotorControl_voltageToDuty(
                vd_command,
                vq_command,

                electrical_angle_rad,

                &duty_u,
                &duty_v,
                &duty_w
            );


            HAL_setPWMDutyFloat(
                duty_u,
                duty_v,
                duty_w
            );
        }


        // ====================================================
        // 10 ms speed loop
        // ====================================================

        if(
            (
                EQEP_getInterruptStatus(
                    ENCODER_EQEP_BASE
                )
                &
                EQEP_INT_UNIT_TIME_OUT
            )
            != 0U
        )
        {
            updateSpeedLoop();


            EQEP_clearInterruptStatus(
                ENCODER_EQEP_BASE,
                EQEP_INT_UNIT_TIME_OUT |
                EQEP_INT_GLOBAL
            );
        }
    }


    HAL_ADC_clearCurrentInterrupt();
}


// ============================================================
// Main
// ============================================================

int main(void)
{
    MotorAppCallbacks app_callbacks;


    // ========================================================
    // Device
    // ========================================================

    Device_init();


    Device_initGPIO();


    DINT;


    Interrupt_initModule();


    Interrupt_initVectorTable();


    // ========================================================
    // Control
    // ========================================================

    MotorControl_init(
        &motor_control,

        (float)MOTOR_POLE_PAIRS,

        ID_PI_KP,
        ID_PI_KI,
        VD_LIMIT,

        IQ_PI_KP,
        IQ_PI_KI,
        VQ_LIMIT,

        SPEED_PI_KP,
        SPEED_PI_KI,
        SPEED_IQ_LIMIT_A,

        SPEED_TO_IQ_SIGN
    );


    // ========================================================
    // GPIO / driver fault interface
    // ========================================================

    HAL_Fault_setupGateEnable();


    HAL_Fault_setupNFAULT();


    HAL_Fault_gateDisable();


    // ========================================================
    // Encoder
    // ========================================================

    HAL_setupEncoderPins();


    HAL_setupEncoder();


    // ========================================================
    // PWM
    // ========================================================

    HAL_setupPWMPins();


    HAL_setupPWMs();


    setNeutralPWM();


    // ========================================================
    // ADC
    // ========================================================

    HAL_ADC_setupCurrentSense();


    HAL_ADC_setupPWMTrigger();


    Interrupt_register(
        INT_ADCA1,
        &adcA1ISR
    );


    Interrupt_enable(
        INT_ADCA1
    );


    // ========================================================
    // SPI / DRV8323
    // ========================================================

    HAL_setupSPI();


    HAL_Fault_gateDisable();


    DEVICE_DELAY_US(
        10000U
    );


    HAL_Fault_gateEnable();


    DEVICE_DELAY_US(
        5000U
    );


    HAL_configureDRV8323RS();


    // ========================================================
    // MotorApp callbacks
    // ========================================================

    app_callbacks.setNeutralPWM =
        setNeutralPWM;


    app_callbacks.gateEnable =
        HAL_Fault_gateEnable;


    app_callbacks.gateDisable =
        HAL_Fault_gateDisable;


    app_callbacks.configureDRV =
        appConfigureDRV;


    app_callbacks.startCurrentCalibration =
        startCurrentCalibration;


    app_callbacks.startAlignment =
        startAlignment;


    app_callbacks.captureAlignmentOffset =
        captureAlignmentOffset;


    app_callbacks.resetRunController =
        appResetRunController;


    MotorApp_init(
        &motor_app,
        &app_callbacks
    );


    // ========================================================
    // Initial commands / diagnostics
    // ========================================================

    motor_command =
        MOTOR_CMD_NONE;


    software_fault_inject =
        0U;


    speed_command_rpm =
        100.0f;


    drv_fault_status1_latched =
        0U;


    drv_fault_status2_latched =
        0U;


    drv_nfault_at_fault =
        1U;


    // ========================================================
    // Interrupts
    // ========================================================

    EINT;


    ERTM;


    // ========================================================
    // 1 ms cooperative application scheduler
    // ========================================================

    while(1)
    {
        MotorAppInputs app_inputs;


        // ====================================================
        // Commands from CCS Watch
        // ====================================================

        motor_app.command =
            motor_command;


        motor_app.software_fault_inject =
            software_fault_inject;


        // ====================================================
        // Real DRV nFAULT
        // ====================================================

        drv_nfault_level =
            HAL_Fault_readNFAULT();


        // ====================================================
        // Capture DRV fault diagnosis BEFORE MotorApp disables
        // the gate driver.
        // ====================================================

        if(
            (drv_nfault_level == 0U)
            &&
            (
                (motor_app.state == MOTOR_STATE_RUN)
                ||
                (
                    motor_app.state ==
                    MOTOR_STATE_FAULT_TEST_READY
                )
            )
        )
        {
            drv_nfault_at_fault =
                0U;


            drv_fault_status1_latched =
                HAL_DRV8323_readRegister(
                    DRV8323_REG_FAULT1
                );


            drv_fault_status2_latched =
                HAL_DRV8323_readRegister(
                    DRV8323_REG_FAULT2
                );
        }


        // ====================================================
        // State-machine inputs
        // ====================================================

        app_inputs.current_calibrated =
            current_calibrated;


        app_inputs.drv_nfault_level =
            drv_nfault_level;


        app_inputs.stop_confirm_counter =
            stop_confirm_counter;


        // ====================================================
        // Application state machine
        // ====================================================

        MotorApp_step(
            &motor_app,
            &app_inputs
        );


        // ====================================================
        // Recovery accepted -> clear diagnostic latch
        // ====================================================

        if(
            (motor_app.state == MOTOR_STATE_RECOVERY_PREP)
            &&
            (motor_app.fault_code == MOTOR_FAULT_NONE)
        )
        {
            drv_fault_status1_latched =
                0U;


            drv_fault_status2_latched =
                0U;


            drv_nfault_at_fault =
                1U;
        }


        // ====================================================
        // CCS mirrors
        // ====================================================

        motor_state =
            motor_app.state;


        motor_fault_code =
            motor_app.fault_code;


        motor_command =
            motor_app.command;


        software_fault_inject =
            motor_app.software_fault_inject;


        // ====================================================
        // 1 ms tick
        // ====================================================

        DEVICE_DELAY_US(
            1000U
        );


        MotorApp_tick1ms(
            &motor_app
        );
    }
}
