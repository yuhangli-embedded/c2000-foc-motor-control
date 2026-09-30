#include "pi_controller.h"


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
// Initialize
// ============================================================

void PIController_init(
    PIController *pi,
    float kp,
    float ki,
    float limit
)
{
    pi->kp =
        kp;


    pi->ki =
        ki;


    pi->limit =
        limit;


    pi->integrator =
        0.0f;


    pi->error =
        0.0f;
}


// ============================================================
// Reset
// ============================================================

void PIController_reset(
    PIController *pi
)
{
    pi->integrator =
        0.0f;


    pi->error =
        0.0f;
}


// ============================================================
// Run
//
// Discrete PI:
//
// u = Kp * e + integral
//
// Conditional integration:
//
// If saturated, integration is only permitted when the
// current error drives the output back toward the linear range.
// ============================================================

float PIController_run(
    PIController *pi,
    float reference,
    float feedback
)
{
    float error;

    float p_term;

    float integrator_candidate;

    float unsat_output;

    float sat_output;


    error =
        reference -
        feedback;


    pi->error =
        error;


    p_term =
        pi->kp *
        error;


    integrator_candidate =
        pi->integrator +
        pi->ki *
        error;


    unsat_output =
        p_term +
        integrator_candidate;


    sat_output =
        clampFloat(
            unsat_output,
            -pi->limit,
            +pi->limit
        );


    // ========================================================
    // Conditional integration anti-windup
    // ========================================================

    if(
        (unsat_output == sat_output)
        ||
        (
            (unsat_output > pi->limit)
            &&
            (error < 0.0f)
        )
        ||
        (
            (unsat_output < -pi->limit)
            &&
            (error > 0.0f)
        )
    )
    {
        pi->integrator =
            integrator_candidate;
    }


    return
        clampFloat(
            p_term +
            pi->integrator,
            -pi->limit,
            +pi->limit
        );
}
