#include "motor_control.h"

#include "clarke_park.h"
#include "voltage_transform.h"


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


// ============================================================
// Initialization
// ============================================================

void MotorControl_init(
    MotorControl *mc,

    float pole_pairs,

    float id_kp,
    float id_ki,
    float vd_limit,

    float iq_kp,
    float iq_ki,
    float vq_limit,

    float speed_kp,
    float speed_ki,
    float speed_iq_limit,

    float speed_to_iq_sign
)
{
    mc->pole_pairs =
        pole_pairs;


    mc->speed_to_iq_sign =
        speed_to_iq_sign;


    PIController_init(
        &mc->id_pi,
        id_kp,
        id_ki,
        vd_limit
    );


    PIController_init(
        &mc->iq_pi,
        iq_kp,
        iq_ki,
        vq_limit
    );


    PIController_init(
        &mc->speed_pi,
        speed_kp,
        speed_ki,
        speed_iq_limit
    );
}


// ============================================================
// Reset
// ============================================================

void MotorControl_reset(
    MotorControl *mc
)
{
    PIController_reset(
        &mc->id_pi
    );


    PIController_reset(
        &mc->iq_pi
    );


    PIController_reset(
        &mc->speed_pi
    );
}


// ============================================================
// Electrical angle
//
// IMPORTANT:
//
// This sign is experimentally verified on the real motor:
//
// theta_e = -p * theta_m + offset
//
// Do NOT change to +p.
// ============================================================

float MotorControl_getElectricalAngleDeg(
    const MotorControl *mc,
    float mechanical_angle_deg,
    float electrical_offset_deg
)
{
    return
        wrap360(
            (
                -mc->pole_pairs *
                mechanical_angle_deg
            )
            +
            electrical_offset_deg
        );
}


// ============================================================
// Current loop
// ============================================================

void MotorControl_runCurrentLoop(
    MotorControl *mc,

    float current_u,
    float current_v,
    float current_w,

    float electrical_angle_rad,

    float id_ref,
    float iq_ref,

    float *current_alpha,
    float *current_beta,

    float *current_d,
    float *current_q,

    float *id_error,
    float *iq_error,

    float *vd_command,
    float *vq_command
)
{
    float sin_theta;

    float cos_theta;


    // ========================================================
    // Electrical angle sin/cos
    // ========================================================

    sin_theta =
        __sin(
            electrical_angle_rad
        );


    cos_theta =
        __cos(
            electrical_angle_rad
        );


    // ========================================================
    // Clarke
    // ========================================================

    MC_clarke(
        current_u,
        current_v,
        current_w,
        current_alpha,
        current_beta
    );


    // ========================================================
    // Park
    // ========================================================

    MC_park(
        *current_alpha,
        *current_beta,
        sin_theta,
        cos_theta,
        current_d,
        current_q
    );


    // ========================================================
    // Id PI
    // ========================================================

    *vd_command =
        PIController_run(
            &mc->id_pi,
            id_ref,
            *current_d
        );


    *id_error =
        mc->id_pi.error;


    // ========================================================
    // Iq PI
    // ========================================================

    *vq_command =
        PIController_run(
            &mc->iq_pi,
            iq_ref,
            *current_q
        );


    *iq_error =
        mc->iq_pi.error;
}


// ============================================================
// Speed loop
// ============================================================

void MotorControl_runSpeedLoop(
    MotorControl *mc,

    float speed_ref_rpm,
    float speed_feedback_rpm,

    float *speed_error_rpm,
    float *torque_command_A,
    float *iq_ref_A
)
{
    *torque_command_A =
        PIController_run(
            &mc->speed_pi,
            speed_ref_rpm,
            speed_feedback_rpm
        );


    *speed_error_rpm =
        mc->speed_pi.error;


    *iq_ref_A =
        mc->speed_to_iq_sign *
        (*torque_command_A);
}


// ============================================================
// dq voltage -> duty
// ============================================================

void MotorControl_voltageToDuty(
    float vd_command,
    float vq_command,

    float electrical_angle_rad,

    float *duty_u,
    float *duty_v,
    float *duty_w
)
{
    float sin_theta;

    float cos_theta;

    float v_alpha;

    float v_beta;

    float v_u;

    float v_v;

    float v_w;


    sin_theta =
        __sin(
            electrical_angle_rad
        );


    cos_theta =
        __cos(
            electrical_angle_rad
        );


    // ========================================================
    // Inverse Park
    // ========================================================

    MC_invPark(
        vd_command,
        vq_command,
        sin_theta,
        cos_theta,
        &v_alpha,
        &v_beta
    );


    // ========================================================
    // Inverse Clarke
    // ========================================================

    MC_invClarke(
        v_alpha,
        v_beta,
        &v_u,
        &v_v,
        &v_w
    );


    // ========================================================
    // Phase command -> duty
    // ========================================================

    MC_phaseVoltageToDuty(
        v_u,
        v_v,
        v_w,
        duty_u,
        duty_v,
        duty_w
    );
}
