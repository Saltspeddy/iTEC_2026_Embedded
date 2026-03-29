#ifndef PWM_MOTOR_TEST_PWM_MOTORS_H
#define PWM_MOTOR_TEST_PWM_MOTORS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

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
void Rotate_90_degrees(motor_dir_t rotate_dir);
void Rotate_180_degrees(void);
void Motor_Stop(void);

#endif
