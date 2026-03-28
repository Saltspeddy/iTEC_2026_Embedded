#ifndef PWM_MOTOR_TEST_PWM_MOTORS_H
#define PWM_MOTOR_TEST_PWM_MOTORS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 *  Strategy
 *  --------
 *  Base speed lives at 490/500 (the only usable slice for 5 V).
 *  We measure left & right wall distances, compute the error
 *  (how far off-centre the robot is), scale it to a small
 *  PWM correction, then add/subtract from each motor.
 *
 *  Correction sign convention:
 *    error > 0  → robot too close to LEFT wall
 *                 → slow left motor, speed up right motor
 *    error < 0  → robot too close to RIGHT wall
 *                 → slow right motor, speed up left motor
 */
#define PWM_MIN         460   /* stall threshold — never go below */
#define PWM_MAX_SPEED   500   /* timer period                    */
#define PWM_BASE        (PWM_MAX_SPEED + (PWM_MAX_SPEED - PWM_MIN) / 2)   /* cruise speed (within 480-500) */

/*
 * How aggressively to correct.
 * correction = (distLeft - distRight) / WALL_GAIN
 * With WALL_GAIN = 3 and a 6 cm side-to-side difference
 * you get a ±2 PWM nudge — safe for the narrow 480-500 range.
 * Tune this first; raise it if the robot still drifts.
 */
#define WALL_GAIN       5.0f

/*
 * Maximum PWM correction applied to either motor.
 * Keeps us from saturating out of the usable band.
 */
#define MAX_CORRECTION  ((PWM_MAX_SPEED - PWM_MIN)  / 2)

/*
 * Distance from a side wall considered "no wall present".
 * Beyond this the sensor reading is ignored for that side.
 */
#define WALL_DETECT_CM  30.0f

/*
 * Obstacle distance: stop / turn below this.
 */
#define OBSTACLE_CM     5.0f

#define PWM_MAX 499

typedef enum {
    LEFT_MOTOR = 3,
    RIGHT_MOTOR = 4,
    BOTH_MOTORS = 5
} motor_t;

typedef enum {
    LEFT_DIR = 6,
    FORWARD_DIR = 7,
    RIGHT_DIR = 8
} motor_dir_t;

extern motor_dir_t currentMotorDir;

void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(motor_t motor, uint16_t speed);
void Motor_ChangeDirection(motor_dir_t dir);
/**
 * @brief  Compute per-motor PWM values that keep the robot
 *         centred between the side walls.
 *
 * @param  distLeft   Distance reported by left  HC-SR04 (cm)
 * @param  distRight  Distance reported by right HC-SR04 (cm)
 * @param  pwmLeft    Output: PWM for left  motor
 * @param  pwmRight   Output: PWM for right motor
 */
void WallCorrection_Compute(float distLeft, float distRight, uint16_t *pwmLeft, uint16_t *pwmRight);
int clamp_int(int v, int lo, int hi);

#endif