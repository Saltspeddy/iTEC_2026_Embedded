#include "maze.h"
#include <string.h>
#include <stdio.h>          // fixes snprintf warning

uint8_t maze[MAZE_ROWS][MAZE_COLS];

void Maze_Init(void)
{
    memset(maze, 0, sizeof(maze));
}

void Maze_OpenWall(uint8_t row, uint8_t col, uint8_t wall_bit)
{
    maze[row][col] |= wall_bit;

    switch (wall_bit) {
        case WALL_N: if (row > 0)           maze[row-1][col]   |= WALL_S; break;
        case WALL_S: if (row < MAZE_ROWS-1) maze[row+1][col]   |= WALL_N; break;
        case WALL_E: if (col < MAZE_COLS-1) maze[row][col+1]   |= WALL_W; break;
        case WALL_W: if (col > 0)           maze[row][col-1]   |= WALL_E; break;
    }
}

// BFS internals
static MazePos_t bfs_queue[MAZE_ROWS * MAZE_COLS];
static MazePos_t bfs_parent[MAZE_ROWS][MAZE_COLS];
static uint8_t   bfs_visited[MAZE_ROWS][MAZE_COLS];

uint8_t Maze_FindPath(MazePos_t start, MazePos_t end,
                      MazePos_t* path_out, uint8_t* path_len_out)
{
    memset(bfs_visited, 0, sizeof(bfs_visited));
    bfs_parent[start.row][start.col] = start;

    uint16_t head = 0, tail = 0;
    bfs_queue[tail++] = start;
    bfs_visited[start.row][start.col] = 1;

    while (head < tail)
    {
        MazePos_t cur = bfs_queue[head++];

        if (cur.row == end.row && cur.col == end.col)
        {
            uint8_t len = 0;
            MazePos_t step = end;
            MazePos_t tmp_path[MAZE_ROWS * MAZE_COLS];

            while (!(step.row == start.row && step.col == start.col))
            {
                tmp_path[len++] = step;
                step = bfs_parent[step.row][step.col];
            }
            tmp_path[len++] = start;

            for (uint8_t i = 0; i < len; i++)
                path_out[i] = tmp_path[len - 1 - i];

            *path_len_out = len;
            return 1;
        }

        uint8_t val = maze[cur.row][cur.col];

        if ((val & WALL_N) && cur.row > 0 && !bfs_visited[cur.row-1][cur.col]) {
            MazePos_t next = {cur.row-1, cur.col};
            bfs_visited[next.row][next.col] = 1;
            bfs_parent[next.row][next.col] = cur;
            bfs_queue[tail++] = next;
        }
        if ((val & WALL_S) && cur.row < MAZE_ROWS-1 && !bfs_visited[cur.row+1][cur.col]) {
            MazePos_t next = {cur.row+1, cur.col};
            bfs_visited[next.row][next.col] = 1;
            bfs_parent[next.row][next.col] = cur;
            bfs_queue[tail++] = next;
        }
        if ((val & WALL_E) && cur.col < MAZE_COLS-1 && !bfs_visited[cur.row][cur.col+1]) {
            MazePos_t next = {cur.row, cur.col+1};
            bfs_visited[next.row][next.col] = 1;
            bfs_parent[next.row][next.col] = cur;
            bfs_queue[tail++] = next;
        }
        if ((val & WALL_W) && cur.col > 0 && !bfs_visited[cur.row][cur.col-1]) {
            MazePos_t next = {cur.row, cur.col-1};
            bfs_visited[next.row][next.col] = 1;
            bfs_parent[next.row][next.col] = cur;
            bfs_queue[tail++] = next;
        }
    }

    *path_len_out = 0;
    return 0;
}

// huart passed in — fixes 'huart2 undeclared' error
// 0xFF marker defined inline — fixes 'PATH_MARKER undeclared' error
void Maze_TransmitWithPath(UART_HandleTypeDef* huart,
                           MazePos_t* path, uint8_t path_len)
{
    char buf[8];
    int  len;

    // build overlay copy
    uint8_t tmp[MAZE_ROWS][MAZE_COLS];
    memcpy(tmp, maze, sizeof(maze));

    for (uint8_t i = 0; i < path_len; i++)
        tmp[path[i].row][path[i].col] = 0xFF;

    for (uint8_t row = 0; row < MAZE_ROWS; row++)
    {
        for (uint8_t col = 0; col < MAZE_COLS; col++)
        {
            len = snprintf(buf, sizeof(buf), "%3d", tmp[row][col]);
            HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void Maze_Print(UART_HandleTypeDef* huart)
{
    char buf[8];
    int len;

    for (uint8_t row = 0; row < MAZE_ROWS; row++)
    {
        for (uint8_t col = 0; col < MAZE_COLS; col++)
        {
            len = snprintf(buf, sizeof(buf), "%3d", maze[row][col]);
            HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}