#include "maze.h"
#include <string.h>

uint8_t maze[MAZE_ROWS][MAZE_COLS];

/* Zero out the grid — all walls closed */
void Maze_Init(void)
{
    memset(maze, 0, sizeof(maze));
}

/*
 * Open a wall and mirror it into the neighbour cell.
 * Always call this instead of writing maze[][] directly
 * so both sides of every passage stay in sync.
 */
void Maze_OpenWall(uint8_t row, uint8_t col, uint8_t wall_bit)
{
    maze[row][col] |= wall_bit;

    /* Mirror into the adjacent cell */
    switch (wall_bit) {
        case WALL_N: if (row > 0)            maze[row-1][col]   |= WALL_S; break;
        case WALL_S: if (row < MAZE_ROWS-1)  maze[row+1][col]   |= WALL_N; break;
        case WALL_E: if (col < MAZE_COLS-1)  maze[row][col+1]   |= WALL_W; break;
        case WALL_W: if (col > 0)            maze[row][col-1]   |= WALL_E; break;
    }
}