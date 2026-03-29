#include "mapping.h"
#include "maze.h"
#include "HCSR04.h"
#include "pwm_motors.h"
#include <string.h>
#include "YS-27.h"
#include "main.h"

/* -----------------------------------------------------------------------
 * Physical constants — tune these to your robot
 * ----------------------------------------------------------------------- */
#define MOVE_SPEED              30U
#define MOVE_TIME_MS            700
#define WALL_OPEN_THRESHOLD_CM  13.0f

/* -----------------------------------------------------------------------
 * Internal state machine
 * ----------------------------------------------------------------------- */
typedef enum {
    MAP_SETTLE = 0,   /* waiting for sensor readings to stabilise */
    MAP_SENSE,        /* read sensors, record walls, decide next move */
    MAP_ROTATE,       /* rotating to face the chosen direction */
    MAP_MOVE,         /* moving one cell forward */
    MAP_DONE          /* entire reachable maze explored */
} map_phase_t;

/* Simple DFS stack — max depth = number of cells */
#define STACK_SIZE (MAZE_ROWS * MAZE_COLS)

typedef struct {
    uint8_t   row;
    uint8_t   col;
    heading_t heading;      /* heading when we arrived here */
    uint8_t   tried;        /* bitmask of absolute dirs already attempted
                               bit0=N, bit1=E, bit2=S, bit3=W */
} stack_frame_t;

static stack_frame_t  dfs_stack[STACK_SIZE];
static int8_t         stack_top = -1;

static uint8_t        visited[MAZE_ROWS][MAZE_COLS];
static map_phase_t    phase          = MAP_DONE;
static heading_t      cur_heading;
static uint8_t        cur_row;
static uint8_t        cur_col;
static heading_t      target_heading; /* heading we need to face before moving */
static uint32_t       phase_start_ms;
static uint8_t        mapping_done   = 0;

/* -----------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */

/* Convert absolute heading to the WALL_x bit */
static uint8_t heading_to_wall(heading_t h)
{
    switch (h) {
        case HEADING_N: return WALL_N;
        case HEADING_E: return WALL_E;
        case HEADING_S: return WALL_S;
        case HEADING_W: return WALL_W;
        default:        return 0;
    }
}

/* Neighbour cell in direction h, returns 0 if out of bounds */
static uint8_t neighbour(uint8_t row, uint8_t col, heading_t h,
                         uint8_t *nr, uint8_t *nc)
{
    switch (h) {
        case HEADING_N: if (row == 0)             return 0; *nr=row-1; *nc=col;   break;
        case HEADING_S: if (row >= MAZE_ROWS-1)   return 0; *nr=row+1; *nc=col;   break;
        case HEADING_E: if (col >= MAZE_COLS-1)   return 0; *nr=row;   *nc=col+1; break;
        case HEADING_W: if (col == 0)             return 0; *nr=row;   *nc=col-1; break;
        default: return 0;
    }
    return 1;
}

/*
 * Map a sensor (FRONT=cur_heading, LEFT=cur_heading-90, RIGHT=cur_heading+90)
 * to the absolute heading it points at.
 */
static heading_t sensor_to_abs(ultrasonicDir_t sensor)
{
    int delta = 0;
    if (sensor == LEFT)  delta = -1;   /* CCW 90° */
    if (sensor == RIGHT) delta =  1;   /* CW  90° */
    return (heading_t)((cur_heading + delta + 4) % 4);
}

/* Rotate the robot to face `target`, updating cur_heading */
static void do_rotate_to(heading_t target)
{
    /* Clockwise rotation count needed */
    int cw = ((int)target - (int)cur_heading + 4) % 4;

    if (cw == 0) {
        /* already facing right direction */
    } else if (cw == 1) {
        Rotate_90_degrees(RIGHT_DIR);
    } else if (cw == 2) {
        Rotate_180_degrees();
        // Rotate_90_degrees(RIGHT_DIR);
        // Rotate_90_degrees(RIGHT_DIR);
    } else { /* cw == 3, i.e. one CCW turn */
        Rotate_90_degrees(LEFT_DIR);
    }
    cur_heading = target;
}

/*
 * Move exactly one cell forward using hall sensor pulse counting.
 * Returns when done or timeout.
 */
/* In mapping.c — replace the old defines and function */


/* Example: if robot does 243mm/s → 280/243*1000 ≈ 1152ms */

static void Move_OneCell(void)
{
    Motor_ChangeDirection(FORWARD_DIR);
    Motor_SetSpeed(BOTH_MOTORS, MOVE_SPEED);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < MOVE_TIME_MS)
    {
        Ultrasonic_Update();
    }

    Motor_SetSpeed(BOTH_MOTORS, 0);

    /* settle delay — non-blocking */
    uint32_t settle = HAL_GetTick();
    while ((HAL_GetTick() - settle) < 800U)
    {
        Ultrasonic_Update();
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void Mapping_Init(uint8_t start_row, uint8_t start_col, heading_t start_heading)
{
    Maze_Init();
    memset(visited, 0, sizeof(visited));

    cur_row     = start_row;
    cur_col     = start_col;
    cur_heading = start_heading;
    stack_top   = -1;
    mapping_done = 0;

    /* Push starting cell */
    stack_top++;
    dfs_stack[stack_top].row     = cur_row;
    dfs_stack[stack_top].col     = cur_col;
    dfs_stack[stack_top].heading = cur_heading;
    dfs_stack[stack_top].tried   = 0;

    visited[cur_row][cur_col] = 1;

    phase          = MAP_SETTLE;
    phase_start_ms = HAL_GetTick();
}

uint8_t Mapping_IsDone(void)
{
    return mapping_done;
}

/*
 * Mapping_Step() — non-blocking state machine, call from main loop.
 *
 * Phase flow:
 *   MAP_SETTLE  →  MAP_SENSE  →  MAP_ROTATE  →  MAP_MOVE  →  MAP_SETTLE …
 *                                                          ↘  MAP_DONE
 */

int rotateDelayFlag = 0;

void Mapping_Step(void)
{
    if (mapping_done) return;

    switch (phase)
    {
        /* ── SETTLE ──────────────────────────────────────────────────── */
        case MAP_SETTLE:
            Ultrasonic_Update();
            if ((HAL_GetTick() - phase_start_ms) >= SENSOR_SETTLE_MS) {
                if (rotateDelayFlag) {
                    phase = MAP_MOVE;
                    rotateDelayFlag = 0;
                }
                else {
                    phase = MAP_SENSE;
                }
            }
            break;

        /* ── SENSE ───────────────────────────────────────────────────── */
        case MAP_SENSE:
        {
            /*
             * Read the three sensors and record open walls.
             * A direction is OPEN when the sensor reads >= threshold
             * AND the neighbour cell exists within the maze bounds.
             */
            ultrasonicDir_t sensors[3] = { CENTER, LEFT, RIGHT };

            for (int i = 0; i < 3; i++) {
                float dist       = HCSR04_GetDistance(sensors[i]);
                heading_t abs_h  = sensor_to_abs(sensors[i]);
                uint8_t   nr, nc;

                /* 999 = no reading yet, skip. Only trust readings below max range */
                if (dist >= WALL_OPEN_THRESHOLD_CM && dist < 600.0f &&
                    neighbour(cur_row, cur_col, abs_h, &nr, &nc))
                {
                    Maze_OpenWall(cur_row, cur_col, heading_to_wall(abs_h));
                }
                // if (dist >= WALL_OPEN_THRESHOLD_CM &&
                //     neighbour(cur_row, cur_col, abs_h, &nr, &nc))
                // {
                //     Maze_OpenWall(cur_row, cur_col, heading_to_wall(abs_h));
                // }
            }

            /*
             * DFS: find an unvisited neighbour through an open wall.
             * Priority: E → N → W → S (change order to taste).
             */
            stack_frame_t *frame = &dfs_stack[stack_top];
            uint8_t moved = 0;

            heading_t priority[4] = { HEADING_E, HEADING_N,
                                      HEADING_W, HEADING_S };

            for (int i = 0; i < 4; i++) {
                heading_t h = priority[i];
                uint8_t bit = 1u << h;   /* matches heading_t ordering */

                if (frame->tried & bit) continue;           /* already tried */

                uint8_t wall_bit = heading_to_wall(h);
                if (!(maze[cur_row][cur_col] & wall_bit)) continue; /* wall closed */

                uint8_t nr, nc;
                if (!neighbour(cur_row, cur_col, h, &nr, &nc)) continue;
                if (visited[nr][nc]) continue;              /* already visited */

                /* Found a valid next cell */
                frame->tried |= bit;
                visited[nr][nc] = 1;
                target_heading   = h;

                stack_top++;
                dfs_stack[stack_top].row     = nr;
                dfs_stack[stack_top].col     = nc;
                dfs_stack[stack_top].heading = h;
                dfs_stack[stack_top].tried   = 0;

                phase = MAP_ROTATE;
                moved = 1;
                break;
            }

            if (!moved) {
                /* Backtrack */
                stack_top--;
                if (stack_top < 0) {
                    /* Exploration complete */
                    mapping_done = 1;
                    phase        = MAP_DONE;
                    Motor_SetSpeed(BOTH_MOTORS, 0);
                    break;
                }

                /*
                 * Return to the previous cell: face the opposite of the
                 * direction we arrived from, then move one cell forward.
                 */
                heading_t came_from = dfs_stack[stack_top + 1].heading;
                target_heading = (heading_t)((came_from + 2) % 4); /* reverse */
                phase = MAP_ROTATE;
            }
            break;
        }

        /* ── ROTATE ──────────────────────────────────────────────────── */
        case MAP_ROTATE:
            /*
             * Rotation is blocking (Rotate_90_degrees uses a while loop
             * with a timeout). We call it here and immediately advance.
             * If you later want non-blocking rotation, split this phase.
             */
            do_rotate_to(target_heading);
            phase_start_ms = HAL_GetTick();
            rotateDelayFlag = 1;
            phase = MAP_SETTLE; /* settle sensors before next sense */
            // phase = MAP_MOVE;
            break;

        /* ── MOVE ────────────────────────────────────────────────────── */
        case MAP_MOVE:
        {
            /*
             * Update logical position BEFORE moving so that if we're
             * backtracking the stack_top already points to previous cell.
             */
            uint8_t nr, nc;
            if (neighbour(cur_row, cur_col, cur_heading, &nr, &nc)) {
                cur_row = nr;
                cur_col = nc;
            }

            Move_OneCell();

            phase_start_ms = HAL_GetTick();
            phase = MAP_SETTLE; /* settle sensors before next sense */
            break;
        }

        case MAP_DONE:
        default:
            mapping_done = 1;
            break;
    }
}