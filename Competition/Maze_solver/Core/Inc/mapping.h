#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include "../Inc/maze.h"

/*
 * Physical threshold: if a sensor reads LESS than this (cm),
 * the wall is considered CLOSED (blocked).
 * Set to roughly half your cell size.
 * E.g. for a 25 cm cell, use 13. Adjust for your maze.
 */
#define WALL_OPEN_THRESHOLD_CM   13.0f

/*
 * After stopping, wait this long before trusting sensor readings.
 * Must be > 30 ms (one full 3-sensor ultrasonic cycle).
 */
#define SENSOR_SETTLE_MS         500U

/* Robot heading — which absolute direction the front sensor faces */
typedef enum {
    HEADING_N = 0,
    HEADING_E = 1,
    HEADING_S = 2,
    HEADING_W = 3
} heading_t;

void Mapping_Init(uint8_t start_row, uint8_t start_col, heading_t start_heading);
void Mapping_Step(void);   /* call repeatedly from main loop while in MAP_MODE */
uint8_t Mapping_IsDone(void);
void Mapping_RotateTo(heading_t target, heading_t *cur_heading_inout);
void Mapping_MoveOneCell(void);
#endif /* MAPPING_H */