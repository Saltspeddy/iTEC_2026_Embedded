#include "../Inc/YS-27.h"
#include "main.h"

#define MAGNETS_PER_REVOLUTION 4 // How many magnets are attached to each wheel
#define WHEEL_DIAMETER 0.065f
#define PI 3.1415926f

volatile uint32_t pulse_left = 0;
volatile uint32_t pulse_right = 0;

float speedLeft = 0.0f;
float speedRight = 0.0f;

void Hall_sensor_counter(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Hall_sensor_left_Pin)
        pulse_left++;

    if (GPIO_Pin == Hall_sensor_right_Pin)
        pulse_right++;
}

void Speed_Update(float dt) // dt is the update period
{
    uint32_t left, right;

    // Protect from interrupt corruption
    __disable_irq();
    left = pulse_left;
    right = pulse_right;
    pulse_left = 0;
    pulse_right = 0;
    __enable_irq();

    float circumference = PI * WHEEL_DIAMETER;

    float rev_left  = (float)left  / MAGNETS_PER_REVOLUTION;
    float rev_right = (float)right / MAGNETS_PER_REVOLUTION;

    // If called every 1 second → already rev/sec
    speedLeft  = (rev_left  * circumference) / dt;
    speedRight = (rev_right * circumference) / dt;
}

float getSpeed(Wheel_t wheel)
{
    if (wheel == LEFT_WHEEL)
        return speedLeft;
    else
        return speedRight;
}