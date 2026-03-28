#include "main.h"
#include "pwm_motors.h"

/* -----------------------------------------------------------------------
 * Module-private state
 * --------------------------------------------------------------------- */
static uint16_t leftMotorSpeed  = PWM_MIN;
static uint16_t rightMotorSpeed = PWM_MIN;
static TIM_HandleTypeDef *motorTim = NULL;

motor_dir_t currentMotorDir = FORWARD_DIR;

/* -----------------------------------------------------------------------
 * Motor_Init
 * --------------------------------------------------------------------- */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    motorTim = htim;
    HAL_TIM_PWM_Start(motorTim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(motorTim, TIM_CHANNEL_4);
}

/* -----------------------------------------------------------------------
 * Motor_SetSpeed
 * --------------------------------------------------------------------- */
void Motor_SetSpeed(motor_t motor, uint16_t speed)
{
    if (speed > PWM_MAX)
        speed = PWM_MAX;

    switch (motor)
    {
        case LEFT_MOTOR:
            leftMotorSpeed = speed;
            motorTim->Instance->CCR3 = speed;
            break;

        case RIGHT_MOTOR:
            rightMotorSpeed = speed;
            motorTim->Instance->CCR4 = speed;
            break;

        case BOTH_MOTORS:
            leftMotorSpeed = speed;
            rightMotorSpeed = speed;
            motorTim->Instance->CCR3 = speed;
            motorTim->Instance->CCR4 = speed;
            break;
    }
}

/* -----------------------------------------------------------------------
 * Motor_Stop  —  zero both PWM outputs immediately
 * --------------------------------------------------------------------- */
void Motor_Stop(void)
{
    leftMotorSpeed  = 0;
    rightMotorSpeed = 0;
    if (motorTim != NULL)
    {
        motorTim->Instance->CCR3 = 0;
        motorTim->Instance->CCR4 = 0;
    }
}

/* -----------------------------------------------------------------------
 * Motor_ChangeDirection
 *
 * Previous implementation used a blocking HAL_Delay() ramp-down loop.
 * That prevented Ultrasonic_Update() and the PID timer ISR from running
 * during direction changes, causing missed echo captures and PID
 * integral wind-up.
 *
 * New approach:
 *   1. Immediately zero both PWM outputs (non-blocking).
 *   2. Reconfigure the H-bridge direction pins.
 *   3. Leave speed at 0 — the PID controller will ramp up naturally on
 *      its next tick once the direction is confirmed in currentMotorDir.
 *
 * The H-bridge (L298N / L293D type) only needs the direction pins to
 * settle before PWM is re-applied; at low speeds there is no need for
 * an artificial coast-down period.
 * --------------------------------------------------------------------- */
void Motor_ChangeDirection(motor_dir_t dir)
{
    /* Step 1: stop PWM immediately */
    Motor_Stop();

    /* Step 2: set H-bridge direction pins */
    switch (dir)
    {
        case FORWARD_DIR:
            /* Left motor forward, right motor forward */
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
            break;

        case RIGHT_DIR:
            /* Left motor forward, right motor reverse → pivot right */
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
            break;

        case LEFT_DIR:
            /* Left motor reverse, right motor forward → pivot left */
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }

    currentMotorDir = dir;
}
