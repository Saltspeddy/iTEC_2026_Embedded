#include "../Inc/YS-27.h"
#include "main.h"

volatile uint32_t pulse_left = 0;
volatile uint32_t pulse_right = 0;

void Hall_sensor_counter(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Hall_sensor_left_Pin)
        pulse_left++;

    if (GPIO_Pin == Hall_sensor_right_Pin)
        pulse_right++;
}
