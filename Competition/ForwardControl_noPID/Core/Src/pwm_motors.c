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

/**
 * @brief  Compute per-motor PWM values that keep the robot
 *         centred between the side walls.
 *
 * @param  distLeft   Distance reported by left  HC-SR04 (cm)
 * @param  distRight  Distance reported by right HC-SR04 (cm)
 * @param  pwmLeft    Output: PWM for left  motor
 * @param  pwmRight   Output: PWM for right motor
 */
void WallCorrection_Compute(float distLeft,
                                   float distRight,
                                   uint16_t *pwmLeft,
                                   uint16_t *pwmRight)
{
    int correction = 0;

    /* Only correct when both walls are visible */
    if (distLeft < WALL_DETECT_CM && distRight < WALL_DETECT_CM)
    {
        /*
         * Positive error  → left wall is closer
         *                  → we've drifted left
         *                  → need to steer right
         *                     (slow left, speed right)
         */

        // float error = distLeft - distRight;
        // correction  = (int)(error / WALL_GAIN);
        // correction  = clamp_int(correction, -MAX_CORRECTION, MAX_CORRECTION);
        float error = distLeft - distRight;
        float absErr = error < 0 ? -error : error;

        if (absErr > 0.01f)   // avoid division by zero
        {
            // quadratic scaling: small differences get small correction, large differences get bigger corrections
            correction = (int)((error / absErr) * (absErr * absErr) / WALL_GAIN);
        }
        else
        {
            correction = 0;
        }
        correction = clamp_int(correction, -MAX_CORRECTION, MAX_CORRECTION);
    }
    else if (distLeft < WALL_DETECT_CM)
    {
        /*
         * Only left wall visible — hug it gently.
         * A fixed small correction nudges us away.
         */
        correction = 3;
    }
    else if (distRight < WALL_DETECT_CM)
    {
        correction = -3;
    }

    /*
     * Apply: left motor gets  (base - correction)
     *        right motor gets (base + correction)
     *
     * Example: drifted left (correction > 0)
     *   left slows  → turns robot back right
     */
    // if (correction > 0)       // too close to left → slow left only
    // {
    //     *pwmLeft  = (uint16_t)clamp_int(PWM_BASE - (correction * 2), PWM_MIN, PWM_MAX);
    //     *pwmRight = PWM_BASE;
    // }
    // else if (correction < 0)  // too close to right → slow right only
    // {
    //     *pwmLeft  = PWM_BASE;
    //     *pwmRight = (uint16_t)clamp_int(PWM_BASE + (correction * 2), PWM_MIN, PWM_MAX);
    // }
    // else
    // {
    //     *pwmLeft  = PWM_BASE;
    //     *pwmRight = PWM_BASE;
    // }

     *pwmLeft  = (uint16_t)clamp_int(PWM_BASE + correction, PWM_MIN, PWM_MAX);
     *pwmRight = (uint16_t)clamp_int(PWM_BASE - correction, PWM_MIN, PWM_MAX);
}

int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}