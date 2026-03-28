#ifndef MAZE_SOLVER_PID_H
#define MAZE_SOLVER_PID_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * PID_t  —  generic discrete PID controller
 *
 * Usage:
 *   1. Declare a PID_t instance (e.g. for each wheel + one for steering).
 *   2. Call PID_Init() once with your gains and output limits.
 *   3. Call PID_Compute() on every fixed-period tick; it returns the
 *      corrected output value.
 *   4. Call PID_Reset() whenever the controller is taken out of service
 *      (direction change, stop, etc.) to clear integral wind-up.
 * --------------------------------------------------------------------- */

typedef struct {
    /* Tuning gains */
    float Kp;
    float Ki;
    float Kd;

    /* Internal state */
    float integral;
    float prev_error;

    /* Anti-windup / output clamp */
    float output_min;
    float output_max;

    /* Last computed output (read-only for caller) */
    float output;
} PID_t;

/**
 * @brief  Initialise a PID controller.
 * @param  pid         Pointer to the PID instance.
 * @param  Kp, Ki, Kd  Gains.
 * @param  out_min     Minimum allowed output (e.g. 0).
 * @param  out_max     Maximum allowed output (e.g. PWM_MAX = 499).
 */
void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max);

/**
 * @brief  Run one PID iteration.
 * @param  pid       Pointer to the PID instance.
 * @param  setpoint  Desired value (m/s).
 * @param  measured  Current measured value (m/s).
 * @param  dt        Time since last call in seconds.
 * @return           Clamped output value (PWM counts).
 */
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

/**
 * @brief  Reset integrator and derivative memory.
 *         Call this before re-enabling a controller after a pause.
 */
void PID_Reset(PID_t *pid);

#endif /* MAZE_SOLVER_PID_H */
