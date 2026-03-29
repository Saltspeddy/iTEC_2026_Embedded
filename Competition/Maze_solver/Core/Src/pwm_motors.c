#include "main.h"
#include "../Inc/pwm_motors.h"

#define ROTATE_SPEED      490U
#define ROTATE_LEFT_MS    320U
#define ROTATE_RIGHT_MS   320U
#define ROTATE_180_MS     620U

static uint16_t leftMotorSpeed = 0;
static uint16_t rightMotorSpeed = 0;
static TIM_HandleTypeDef *motorTim;

motor_dir_t currentMotorDir = FORWARD_DIR;

void Motor_Init(TIM_HandleTypeDef *htim)
{
    motorTim = htim;
    HAL_TIM_PWM_Start(motorTim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(motorTim, TIM_CHANNEL_4);
}

void Motor_SetSpeed(motor_t motor, uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;

    switch (motor)
    {
        case LEFT_MOTOR:
            leftMotorSpeed = speed;
            motorTim->Instance->CCR4 = speed;
            break;

        case RIGHT_MOTOR:
            rightMotorSpeed = speed;
            motorTim->Instance->CCR3 = speed;
            break;

        case BOTH_MOTORS:
            leftMotorSpeed = speed;
            rightMotorSpeed = speed;
            motorTim->Instance->CCR3 = speed;
            motorTim->Instance->CCR4 = speed;
            break;
    }
}

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

void Motor_ChangeDirection(motor_dir_t dir)
{
    Motor_Stop();
    switch (dir)
    {
        case FORWARD_DIR:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
            break;

        case RIGHT_DIR:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
            break;

        case LEFT_DIR:

            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

            HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
            HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
            break;
    }

    currentMotorDir = dir;
}

void Rotate_90_degrees(motor_dir_t rotate_dir)
{
    uint32_t duration_ms;

    if (rotate_dir == LEFT_DIR)
    {
        duration_ms = ROTATE_LEFT_MS;
    }
    else if (rotate_dir == RIGHT_DIR)
    {
        duration_ms = ROTATE_RIGHT_MS;
    }
    else
    {
        return;
    }

    Motor_SetSpeed(BOTH_MOTORS, 0);
    HAL_Delay(100);

    Motor_ChangeDirection(rotate_dir);
    Motor_SetSpeed(BOTH_MOTORS, ROTATE_SPEED);
    HAL_Delay(duration_ms);
    Motor_SetSpeed(BOTH_MOTORS, 0);

    HAL_Delay(50);
    Motor_ChangeDirection(FORWARD_DIR);
}

void Rotate_180_degrees(void)
{
    Motor_SetSpeed(BOTH_MOTORS, 0);
    HAL_Delay(100);

    Motor_ChangeDirection(LEFT_DIR);
    Motor_SetSpeed(BOTH_MOTORS, ROTATE_SPEED);
    HAL_Delay(ROTATE_180_MS);
    Motor_SetSpeed(BOTH_MOTORS, 0);

    HAL_Delay(50);
    Motor_ChangeDirection(FORWARD_DIR);
}
