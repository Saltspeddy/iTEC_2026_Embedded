//
// Created by dragos on 3/28/26.
//

#ifndef MAZE_SOLVER_YS_27_H
#define MAZE_SOLVER_YS_27_H

#include <stdint.h>

extern volatile uint32_t pulse_left;
extern volatile uint32_t pulse_right;

void Hall_sensor_counter(uint16_t GPIO_Pin);


#endif //MAZE_SOLVER_YS_27_H
