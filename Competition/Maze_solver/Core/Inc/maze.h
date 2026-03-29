#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MAZE_ROWS 10
#define MAZE_COLS 10

#define WALL_N 0x01
#define WALL_E 0x02
#define WALL_S 0x04
#define WALL_W 0x08

typedef struct {
    uint8_t row;
    uint8_t col;
} MazePos_t;

extern uint8_t maze[MAZE_ROWS][MAZE_COLS];

void    Maze_Init(void);
void    Maze_OpenWall(uint8_t row, uint8_t col, uint8_t wall_bit);
uint8_t Maze_FindPath(MazePos_t start, MazePos_t end, MazePos_t* path_out, uint8_t* path_len_out);
void    Maze_TransmitWithPath(UART_HandleTypeDef* huart, MazePos_t* path, uint8_t path_len);
void Maze_Print(UART_HandleTypeDef* huart);

#endif