//
// Created by dragos on 3/28/26.
//

#ifndef MAZE_SOLVER_YS_27_H
#define MAZE_SOLVER_YS_27_H

#include <stdint.h>

typedef enum {
    LEFT_WHEEL = 0,
    RIGHT_WHEEL = 1
} Wheel_t;

extern volatile uint32_t pulse_left;
extern volatile uint32_t pulse_right;

extern float speedLeft;
extern float speedRight;

void Hall_sensor_counter(uint16_t GPIO_Pin);
void Speed_Update(float dt); // dt is the update period
float getSpeed(Wheel_t wheel);

#endif //MAZE_SOLVER_YS_27_H
