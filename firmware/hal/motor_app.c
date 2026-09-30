#include "motor_app.h"


// ============================================================
// Timing
// ============================================================

#define ALIGN_HOLD_MS          800U
#define ALIGN_SETTLE_MS        100U

#define DRV_WAKE_MS            10U
#define DRV_SETTLE_MS          20U


// ============================================================
// State transition
// ============================================================

static void MotorApp_setState(
    MotorApp *app,
    MotorState new_state
)
{
    app->state =
        new_state;


    app->state_time_ms =
        0U;
}


// ============================================================
// Init
// ============================================================

void MotorApp_init(
    MotorApp *app,
    const MotorAppCallbacks *callbacks
)
{
    app->state =
        MOTOR_STATE_INIT;


    app->command =
        MOTOR_CMD_NONE;


    app->fault_code =
        MOTOR_FAULT_NONE;


    app->state_time_ms =
        0U;


    app->current_loop_active =
        0U;


    app->speed_loop_active =
        0U;


    app->software_fault_inject =
        0U;


    app->drv_configured =
        0U;


    app->alignment_capture_done =
        0U;


    app->drv_nfault_at_fault =
        1U;


    app->callbacks =
        *callbacks;
}


// ============================================================
// Fault entry
// ============================================================

void MotorApp_enterFault(
    MotorApp *app,
    MotorFault fault
)
{
    app->fault_code =
        fault;


    app->current_loop_active =
        0U;


    app->speed_loop_active =
        0U;


    app->callbacks.setNeutralPWM();


    app->callbacks.gateDisable();


    MotorApp_setState(
        app,
        MOTOR_STATE_FAULT
    );
}


// ============================================================
// State machine
// ============================================================

void MotorApp_step(
    MotorApp *app,
    const MotorAppInputs *inputs
)
{
    switch(app->state)
    {
        // ====================================================
        // INIT
        // ====================================================

        case MOTOR_STATE_INIT:
        {
            app->callbacks.setNeutralPWM();


            app->fault_code =
                MOTOR_FAULT_NONE;


            app->current_loop_active =
                0U;


            app->speed_loop_active =
                0U;


            app->alignment_capture_done =
                0U;


            app->drv_configured =
                0U;


            app->callbacks.resetRunController();


            app->callbacks.gateEnable();


            app->callbacks.startCurrentCalibration();


            MotorApp_setState(
                app,
                MOTOR_STATE_CURRENT_CAL
            );


            break;
        }


        // ====================================================
        // CURRENT CAL
        // ====================================================

        case MOTOR_STATE_CURRENT_CAL:
        {
            if(inputs->current_calibrated != 0U)
            {
                app->callbacks.startAlignment();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_ALIGN
                );
            }


            break;
        }


        // ====================================================
        // ALIGN
        // ====================================================

        case MOTOR_STATE_ALIGN:
        {
            if(
                (app->alignment_capture_done == 0U)
                &&
                (app->state_time_ms >= ALIGN_HOLD_MS)
            )
            {
                app->callbacks.captureAlignmentOffset();


                app->alignment_capture_done =
                    1U;


                app->callbacks.setNeutralPWM();
            }


            if(
                (app->alignment_capture_done != 0U)
                &&
                (
                    app->state_time_ms >=
                    (ALIGN_HOLD_MS + ALIGN_SETTLE_MS)
                )
            )
            {
                app->callbacks.setNeutralPWM();


                app->callbacks.gateDisable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_READY
                );
            }


            break;
        }


        // ====================================================
        // READY
        // ====================================================

        case MOTOR_STATE_READY:
        {
            app->callbacks.setNeutralPWM();


            app->callbacks.gateDisable();


            // ------------------------------------------------
            // Dedicated hardware-fault test
            // ------------------------------------------------

            if(app->command == MOTOR_CMD_FAULT_TEST)
            {
                app->command =
                    MOTOR_CMD_NONE;


                app->current_loop_active =
                    0U;


                app->speed_loop_active =
                    0U;


                app->callbacks.resetRunController();


                app->drv_configured =
                    0U;


                app->drv_nfault_at_fault =
                    1U;


                app->callbacks.setNeutralPWM();


                app->callbacks.gateEnable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_FAULT_TEST_READY
                );


                break;
            }


            // ------------------------------------------------
            // Normal START
            // ------------------------------------------------

            if(app->command == MOTOR_CMD_START)
            {
                app->command =
                    MOTOR_CMD_NONE;


                app->current_loop_active =
                    0U;


                app->speed_loop_active =
                    0U;


                app->drv_configured =
                    0U;


                app->callbacks.resetRunController();


                app->callbacks.setNeutralPWM();


                app->callbacks.gateEnable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_RUN_PREP
                );
            }


            break;
        }


        // ====================================================
        // FAULT TEST READY
        // ====================================================

        case MOTOR_STATE_FAULT_TEST_READY:
        {
            app->callbacks.setNeutralPWM();


            app->current_loop_active =
                0U;


            app->speed_loop_active =
                0U;


            if(
                (app->drv_configured == 0U)
                &&
                (app->state_time_ms >= DRV_WAKE_MS)
            )
            {
                app->callbacks.configureDRV();


                app->callbacks.setNeutralPWM();


                app->drv_configured =
                    1U;
            }


            if(
                (app->drv_configured != 0U)
                &&
                (inputs->drv_nfault_level == 0U)
            )
            {
                app->drv_nfault_at_fault =
                    0U;


                MotorApp_enterFault(
                    app,
                    MOTOR_FAULT_DRV8323
                );


                break;
            }


            if(app->command == MOTOR_CMD_STOP)
            {
                app->command =
                    MOTOR_CMD_NONE;


                app->callbacks.setNeutralPWM();


                app->callbacks.gateDisable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_READY
                );
            }


            break;
        }


        // ====================================================
        // RUN PREP
        // ====================================================

        case MOTOR_STATE_RUN_PREP:
        {
            app->callbacks.setNeutralPWM();


            if(
                (app->drv_configured == 0U)
                &&
                (app->state_time_ms >= DRV_WAKE_MS)
            )
            {
                app->callbacks.configureDRV();


                app->callbacks.resetRunController();


                app->callbacks.setNeutralPWM();


                app->drv_configured =
                    1U;
            }


            if(
                (app->drv_configured != 0U)
                &&
                (
                    app->state_time_ms >=
                    (DRV_WAKE_MS + DRV_SETTLE_MS)
                )
            )
            {
                app->current_loop_active =
                    1U;


                app->speed_loop_active =
                    1U;


                MotorApp_setState(
                    app,
                    MOTOR_STATE_RUN
                );
            }


            break;
        }


        // ====================================================
        // RUN
        // ====================================================

        case MOTOR_STATE_RUN:
        {
            // ------------------------------------------------
            // Real DRV nFAULT
            // ------------------------------------------------

            if(inputs->drv_nfault_level == 0U)
            {
                app->drv_nfault_at_fault =
                    0U;


                app->command =
                    MOTOR_CMD_NONE;


                MotorApp_enterFault(
                    app,
                    MOTOR_FAULT_DRV8323
                );


                break;
            }


            // ------------------------------------------------
            // Software fault injection
            // ------------------------------------------------

            if(app->software_fault_inject != 0U)
            {
                app->software_fault_inject =
                    0U;


                app->command =
                    MOTOR_CMD_NONE;


                MotorApp_enterFault(
                    app,
                    MOTOR_FAULT_SOFTWARE_TEST
                );


                break;
            }


            // ------------------------------------------------
            // Controlled STOP
            // ------------------------------------------------

            if(app->command == MOTOR_CMD_STOP)
            {
                app->command =
                    MOTOR_CMD_NONE;


                MotorApp_setState(
                    app,
                    MOTOR_STATE_STOP
                );
            }


            break;
        }


        // ====================================================
        // STOP
        // ====================================================

        case MOTOR_STATE_STOP:
        {
            if(inputs->stop_confirm_counter >= 20U)
            {
                app->current_loop_active =
                    0U;


                app->speed_loop_active =
                    0U;


                app->callbacks.setNeutralPWM();


                app->callbacks.gateDisable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_READY
                );
            }


            break;
        }


        // ====================================================
        // FAULT
        // ====================================================

        case MOTOR_STATE_FAULT:
        {
            app->callbacks.setNeutralPWM();


            app->callbacks.gateDisable();


            app->current_loop_active =
                0U;


            app->speed_loop_active =
                0U;

            app->software_fault_inject =
                0U;

            if(app->command == MOTOR_CMD_RESET)
            {
                app->command =
                    MOTOR_CMD_NONE;


                app->software_fault_inject =
                    0U;


                app->callbacks.setNeutralPWM();


                app->callbacks.resetRunController();


                app->fault_code =
                    MOTOR_FAULT_NONE;


                app->alignment_capture_done =
                    0U;


                app->drv_configured =
                    0U;


                app->drv_nfault_at_fault =
                    1U;


                app->callbacks.gateEnable();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_RECOVERY_PREP
                );
            }


            break;
        }


        // ====================================================
        // RECOVERY PREP
        // ====================================================

        case MOTOR_STATE_RECOVERY_PREP:
        {
            app->callbacks.setNeutralPWM();


            if(
                (app->drv_configured == 0U)
                &&
                (app->state_time_ms >= DRV_WAKE_MS)
            )
            {
                app->callbacks.configureDRV();


                app->callbacks.setNeutralPWM();


                app->drv_configured =
                    1U;
            }


            if(
                (app->drv_configured != 0U)
                &&
                (
                    app->state_time_ms >=
                    (DRV_WAKE_MS + DRV_SETTLE_MS)
                )
            )
            {
                app->callbacks.startCurrentCalibration();


                MotorApp_setState(
                    app,
                    MOTOR_STATE_CURRENT_CAL
                );
            }


            break;
        }


        // ====================================================
        // Invalid state
        // ====================================================

        default:
        {
            MotorApp_enterFault(
                app,
                MOTOR_FAULT_DRV8323
            );


            break;
        }
    }
}


// ============================================================
// 1 ms tick
// ============================================================

void MotorApp_tick1ms(
    MotorApp *app
)
{
    app->state_time_ms++;
}
