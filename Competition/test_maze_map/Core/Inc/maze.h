#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

#define MAZE_ROWS 10
#define MAZE_COLS 10

#define WALL_N 0x01
#define WALL_E 0x02
#define WALL_S 0x04
#define WALL_W 0x08

extern uint8_t maze[MAZE_ROWS][MAZE_COLS];

void Maze_Init(void);
void Maze_OpenWall(uint8_t row, uint8_t col, uint8_t wall_bit);

#endif
