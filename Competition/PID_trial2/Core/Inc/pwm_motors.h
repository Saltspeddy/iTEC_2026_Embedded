#ifndef PWM_MOTOR_TEST_PWM_MOTORS_H
#define PWM_MOTOR_TEST_PWM_MOTORS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* -----------------------------------------------------------------------
 * PWM range: TIM3 period = 499, so valid CCR values are 0–499.
 * --------------------------------------------------------------------- */
#define PWM_MAX  499
#define PWM_MIN  490

typedef enum {
    LEFT_MOTOR   = 3,
    RIGHT_MOTOR  = 4,
    BOTH_MOTORS  = 5
} motor_t;

typedef enum {
    LEFT_DIR    = 6,
    FORWARD_DIR = 7,
    RIGHT_DIR   = 8
} motor_dir_t;

/* Current direction — readable by main / other modules */
extern motor_dir_t currentMotorDir;

/* -----------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------- */

/** Initialise PWM channels and start both motor timers. */
void Motor_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  Set a wheel's PWM duty cycle directly.
 * @param  motor  LEFT_MOTOR, RIGHT_MOTOR, or BOTH_MOTORS.
 * @param  speed  0 … PWM_MAX (499).
 */
void Motor_SetSpeed(motor_t motor, uint16_t speed);

/**
 * @brief  Instantly stop both motors and reconfigure the H-bridge
 *         direction pins for the requested direction.
 *
 *         The old implementation ramped down with HAL_Delay() which
 *         blocked the entire main loop (and therefore Ultrasonic_Update,
 *         PID ticks, etc.).  This version stops immediately — the PID
 *         controller will ramp speed back up smoothly on its own.
 *
 * @param  dir  FORWARD_DIR, LEFT_DIR, or RIGHT_DIR.
 */
void Motor_ChangeDirection(motor_dir_t dir);

/**
 * @brief  Hard-stop both motors (speed = 0, does not change direction).
 *         Call this when stopSignal is asserted.
 */
void Motor_Stop(void);

#endif /* PWM_MOTOR_TEST_PWM_MOTORS_H */
