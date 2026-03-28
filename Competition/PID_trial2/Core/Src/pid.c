#include "pid.h"

/* -----------------------------------------------------------------------
 * Helper: clamp a float value between [min, max]
 * --------------------------------------------------------------------- */
static float clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* -----------------------------------------------------------------------
 * PID_Init
 * --------------------------------------------------------------------- */
void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max)
{
    pid->Kp         = Kp;
    pid->Ki         = Ki;
    pid->Kd         = Kd;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/* -----------------------------------------------------------------------
 * PID_Compute
 *
 * Implements a standard positional PID with:
 *   - Integral clamping (anti-windup): the integral term is itself
 *     clamped to [out_min, out_max] so it cannot wind up beyond what
 *     the actuator can deliver.
 *   - Output clamping: final output is hard-clamped before returning.
 *   - Derivative on measurement (not on error): avoids derivative kick
 *     when the setpoint changes suddenly. We negate prev_error usage
 *     accordingly — see note below.
 *
 * NOTE on derivative direction:
 *   Classic form:  d_term = Kd * (error - prev_error) / dt
 *   We keep this form because the setpoint is constant during straight-
 *   line driving. If you later add dynamic setpoints, switch to
 *   derivative-on-measurement: d_term = -Kd * (measured - prev_measured) / dt
 * --------------------------------------------------------------------- */
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f) return pid->output;   /* Guard against division by zero */

    float error      = setpoint - measured;

    /* --- Proportional --- */
    float p_term     = pid->Kp * error;

    /* --- Integral with anti-windup clamp --- */
    pid->integral   += error * dt;
    /* Clamp the accumulated integral to prevent windup */
    pid->integral    = clamp(pid->integral,
                             pid->output_min / (pid->Ki > 0.0f ? pid->Ki : 1.0f),
                             pid->output_max / (pid->Ki > 0.0f ? pid->Ki : 1.0f));
    float i_term     = pid->Ki * pid->integral;

    /* --- Derivative --- */
    float d_term     = pid->Kd * (error - pid->prev_error) / dt;
    // float d_term     = -pid->Kd * (measured - prev_measured) / dt;
    pid->prev_error  = error;

    /* --- Sum and clamp output --- */
    pid->output = clamp(p_term + i_term + d_term,
                        pid->output_min,
                        pid->output_max);

    return pid->output;
}

/* -----------------------------------------------------------------------
 * PID_Reset
 * --------------------------------------------------------------------- */
void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}
