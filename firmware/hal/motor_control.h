#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H


#include "pi_controller.h"


typedef struct
{
    float pole_pairs;

    float speed_to_iq_sign;


    PIController id_pi;

    PIController iq_pi;

    PIController speed_pi;

} MotorControl;


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
);


// ============================================================
// Reset all PI internal states
// ============================================================

void MotorControl_reset(
    MotorControl *mc
);


// ============================================================
// Mechanical angle -> electrical angle
//
// Verified project relationship:
//
// theta_e = -p * theta_m + offset
// ============================================================

float MotorControl_getElectricalAngleDeg(
    const MotorControl *mc,
    float mechanical_angle_deg,
    float electrical_offset_deg
);


// ============================================================
// Current-loop calculation
//
// Phase current
//   -> Clarke
//   -> Park
//   -> Id/Iq PI
//
// No hardware PWM update is performed here.
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
);


// ============================================================
// Speed PI
//
// speed reference
//      -> Speed PI
//      -> torque-current command
//      -> Iq reference
// ============================================================

void MotorControl_runSpeedLoop(
    MotorControl *mc,

    float speed_ref_rpm,
    float speed_feedback_rpm,

    float *speed_error_rpm,
    float *torque_command_A,
    float *iq_ref_A
);


// ============================================================
// dq voltage -> 3-phase PWM duty
//
// InvPark
// -> InvClarke
// -> duty conversion
//
// Does NOT write hardware PWM.
// ============================================================

void MotorControl_voltageToDuty(
    float vd_command,
    float vq_command,

    float electrical_angle_rad,

    float *duty_u,
    float *duty_v,
    float *duty_w
);


#endif
