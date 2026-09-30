#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H


typedef struct
{
    float kp;
    float ki;
    float limit;

    float integrator;
    float error;

} PIController;


// ============================================================
// Initialize PI controller
// ============================================================

void PIController_init(
    PIController *pi,
    float kp,
    float ki,
    float limit
);


// ============================================================
// Clear controller internal state
// ============================================================

void PIController_reset(
    PIController *pi
);


// ============================================================
// Execute one discrete PI step
//
// Includes conditional-integration anti-windup.
// ============================================================

float PIController_run(
    PIController *pi,
    float reference,
    float feedback
);


#endif
