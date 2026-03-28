#include "main.h"
#include "../Inc/pwm_motors.h"

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
