#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "receiver.h"

#define MAZE_SIZE 16
#define MAZE_HEIGHT 16
#define MAZE_WIDTH 16
#define MAX_COST (MAZE_HEIGHT * MAZE_WIDTH - 1)

#define NORTH_WALL 0b00001000
#define EAST_WALL  0b00000100
#define SOUTH_WALL 0b00000010
#define WEST_WALL  0b00000001

const char *MAZE_TAG = "maze";

typedef enum { NORTH = 0, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST, HEADING_COUNT, BLOCKED} Heading;

inline Heading right90_from(const Heading heading) {
    return (Heading)((heading + 2) % HEADING_COUNT);
}

inline Heading right45_from(const Heading heading) {
    return (Heading)((heading + 1) % HEADING_COUNT);
}

inline Heading left90_from(const Heading heading) {
    return (Heading)((heading + HEADING_COUNT - 2) % HEADING_COUNT);
}

inline Heading left45_from(const Heading heading) {
    return (Heading)((heading + HEADING_COUNT - 1) % HEADING_COUNT);
}

inline Heading ahead_from(const Heading heading) {
  return heading;
}

inline Heading behind_from(const Heading heading) {
  return (Heading)((heading + HEADING_COUNT / 2) % HEADING_COUNT);
}

typedef enum {
    ABSENT = 0,
    WALL = 1,
    UNKNOWN = 2,
    VIRTUAL = 3
} WallState;

typedef struct {
    WallState north : 2;
    WallState east : 2;
    WallState south : 2;
    WallState west : 2;
} WallInfo;

typedef enum { AHEAD, RIGHT, BACK, LEFT } Direction;

typedef enum {
  MASK_OPEN = 0x01,    // used for search
  MASK_CLOSED = 0x03  // used for speed
} MazeMask;

/* Position in the maze as an (x,y) coordinate pair. */
typedef struct {
    uint8_t x;
    uint8_t y;
} Pos;

bool is_in_maze(Pos pos) {
    return pos.x < MAZE_WIDTH && pos.y < MAZE_HEIGHT;
}

Pos neighbor(Pos pos, Heading heading) {
    Pos neighbor_pos = pos;
    switch (heading) {
        case NORTH:
            neighbor_pos.y = pos.y + 1;
            break;
        case EAST:
            neighbor_pos.x = pos.x + 1;
            break;
        case SOUTH:
            neighbor_pos.y = pos.y - 1;
            break;
        case WEST:
            neighbor_pos.x = pos.x - 1;
            break;
        default:
            return (Pos){0xFF, 0xFF};
            break;
    }
    if (is_in_maze(neighbor_pos))
        return neighbor_pos;
    else 
        return (Pos){0xFF, 0xFF};
}

typedef struct {
	uint8_t m_cost[MAZE_SIZE][MAZE_SIZE];
	WallInfo m_walls[MAZE_SIZE][MAZE_SIZE];

	Pos m_mouse_pos;
	Heading m_mouse_heading;

	Pos m_goal;

    MazeMask m_mask;
} Maze;

Maze maze;

// Maze Functions
void initializeMaze(Maze* maze, Pos start);
void destroyMaze(Maze* maze);

// void set_goal_pos();

void reset_maze(Maze* maze) {
    maze->m_mouse_pos = (Pos){0, 0};
    maze->m_mouse_heading = NORTH;

    for (uint8_t row = 0; row < MAZE_HEIGHT; row++) {
        for (uint8_t col = 0; col < MAZE_WIDTH; col++) {
            if (row > 0) {
                maze->m_walls[row][col].south = UNKNOWN;
            } else {
                maze->m_walls[row][col].south = WALL;
            }

            if (row < MAZE_HEIGHT - 1) {
                maze->m_walls[row][col].north = UNKNOWN;
            } else {
                maze->m_walls[row][col].north = WALL;
            }

            if (col > 0) {
                maze->m_walls[row][col].west = UNKNOWN;
            } else {
                maze->m_walls[row][col].west = WALL;
            }

            if (col < MAZE_WIDTH - 1) {
                maze->m_walls[row][col].east = UNKNOWN;
            } else {
                maze->m_walls[row][col].east = WALL;
            }
        }
    }
}

bool is_available(Maze* maze, Pos cell, Heading heading) {
    bool result = false;
    WallInfo walls = maze->m_walls[cell.y][cell.x];
    switch (heading) {
      case NORTH:
        result = (walls.north & maze->m_mask) == ABSENT;
        break;
      case EAST:
        result = (walls.east & maze->m_mask) == ABSENT;
        break;
      case SOUTH:
        result = (walls.south & maze->m_mask) == ABSENT;
        break;
      case WEST:
        result = (walls.west & maze->m_mask) == ABSENT;
        break;
      default:
        result = false;
        break;
    }
    return result;
  }

/// @brief Floods maze with correct cost values for each cell
void flood(Maze* maze) {
    for (uint8_t i = 0; i < MAZE_WIDTH; i++) {
		for (uint8_t j = 0; j < MAZE_HEIGHT; j++) {
			maze->m_cost[j][i] = MAX_COST;
		}
	}

    // for (uint8_t i = 0; i < maze->m_num_goals; i++) {
		// maze->m_cost[maze->m_goals[i].y][maze->m_goals[i].x] = 0;
	// }
    maze->m_cost[maze->m_goal.y][maze->m_goal.x] = 0;

	uint8_t head = 0;
	uint8_t tail = 0;
	Pos queue[MAX_COST];
    
	// queue[tail] = maze->m_goals[0];
    queue[tail] = maze->m_goal;
	tail++;

    ESP_LOGI(MAZE_TAG, "floodfill start");
    while (tail - head > 0)	{
		Pos here = queue[head];
        head++;
        uint16_t newCost = maze->m_cost[here.y][here.x] + 1;

        for (uint8_t h = NORTH; h < HEADING_COUNT; h += 2) {
            Heading heading = (Heading)(h);
            if (is_available(maze, here, heading)) {
                Pos nextCell = neighbor(here, heading);
                    if (is_in_maze(nextCell) && maze->m_cost[nextCell.y][nextCell.x] > newCost) {
                        maze->m_cost[nextCell.y][nextCell.x] = newCost;
                        queue[tail] = nextCell;
                        tail++;
                }
            }
        }
    }
    ESP_LOGI(MAZE_TAG, "floodfill end");
}

/// @brief returns the heading with the smallest neighboring cost value 
Heading heading_to_smallest(Maze* maze, Pos pos, Heading start_heading)
{
    Heading next_heading = start_heading;
    Heading best_heading = BLOCKED;
    uint8_t best_cost = maze->m_cost[pos.y][pos.x];
    uint8_t cost = MAX_COST;
    Pos next_cell;

    if (is_available(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        if (is_in_maze(next_cell)) {
            cost = maze->m_cost[next_cell.y][next_cell.x];
            if (cost < best_cost) {
                best_cost = cost;
                best_heading = next_heading;
            }
        }
    }

    next_heading = right90_from(start_heading);
    if (is_available(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        if (is_in_maze(next_cell)) {
            cost = maze->m_cost[next_cell.y][next_cell.x];
            if (cost < best_cost) {
                best_cost = cost;
                best_heading = next_heading;
            }
        }
    }

    next_heading = left90_from(start_heading);
    if (is_available(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        if (is_in_maze(next_cell)) {
            cost = maze->m_cost[next_cell.y][next_cell.x];
            if (cost < best_cost) {
                best_cost = cost;
                best_heading = next_heading;
            }
        }
    }

    next_heading = behind_from(start_heading);
    if (is_available(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        if (is_in_maze(next_cell)) {
            cost = maze->m_cost[next_cell.y][next_cell.x];
            if (cost < best_cost) {
                best_cost = cost;
                best_heading = next_heading;
            }
        }
    }

    return best_heading;
}

/// @brief returns true if any walls in a cell have not been seen
bool has_unknown_walls(Maze* maze, Pos cell) {
    WallInfo walls_here = maze->m_walls[cell.y][cell.x];
    if (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN || walls_here.south == UNKNOWN || walls_here.west == UNKNOWN) {
        return true;
    } else {
        return false;
    }
}

/// @brief returns true if all the walls of the cell have been seen, false otherwise
bool cell_is_visited(Maze* maze, Pos cell) {
    return !has_unknown_walls(maze, cell);
}

void print_maze_state(Maze* maze) {
	char buffer[300] = {'\0'};

    strcpy(buffer, "+");

	for (uint8_t x = 0; x < MAZE_WIDTH; x++)
	{
		if (maze->m_walls[MAZE_HEIGHT - 1][x].north == WALL)
			strcat(buffer, "---+");
		else
			strcat(buffer, "   +");
	}
    
    ESP_LOGI(MAZE_TAG, "%s", buffer);

    
	for(int8_t y = MAZE_HEIGHT - 1; y >= 0; y--)
	{
		strcpy(buffer, "|");
		for (uint8_t x = 0; x < MAZE_WIDTH; x++)
		{
			char num_buf[6];
			if (x == maze->m_mouse_pos.x && y == maze->m_mouse_pos.y)
			{
				if(maze->m_mouse_heading == NORTH) {
					sprintf(num_buf, " N ");
				} else if (maze->m_mouse_heading == SOUTH) {
					sprintf(num_buf, " S ");
				} else if (maze->m_mouse_heading == WEST) {
					sprintf(num_buf, " W ");
				} else if (maze->m_mouse_heading == EAST) {
					sprintf(num_buf, " E ");
				} else {
				sprintf(num_buf, "%3d", maze->m_cost[y][x]);
                }

				if (maze->m_walls[y][x].east == WALL)
				{
					strcat(num_buf, "|");
				}
				else
					strcat(num_buf, " ");
			}
			else if (maze->m_walls[y][x].east == WALL)
			{
				sprintf(num_buf, "%3d|", maze->m_cost[y][x]);
			}
			else
			{
				sprintf(num_buf, "%3d ", maze->m_cost[y][x]);
			}
			strcat(buffer, num_buf);
		}

        ESP_LOGI(MAZE_TAG, "%s", buffer);
	
		strcpy(buffer, "+");
		for (uint8_t x = 0; x < MAZE_SIZE; x++)
		{
			if (maze->m_walls[y][x].south == WALL)
				strcat(buffer, "---+");
			else
				strcat(buffer, "   +");
		}
        ESP_LOGI(MAZE_TAG, "%s", buffer);
    }
}

void scan_new_walls(Maze* maze) {
    uint8_t x = maze->m_mouse_pos.x;
	uint8_t y = maze->m_mouse_pos.y;

    bool left_wall = left_wall_present;
    bool right_wall = right_wall_present;
    bool front_wall = front_wall_present;
        
	switch (maze->m_mouse_heading)
	{
        case NORTH:
            if (left_wall) {
                maze->m_walls[y][x].west = WALL;
                if (x > 0) {
                    maze->m_walls[y][x - 1].east = WALL;
                }
            }
            if (right_wall) {
                maze->m_walls[y][x].east = WALL;
                if (x < MAZE_HEIGHT - 1) {
                    maze->m_walls[y][x + 1].west = WALL;
                }
            }
            if (front_wall) {
                maze->m_walls[y][x].north = WALL;
                if (y < MAZE_HEIGHT - 1) {   
                    maze->m_walls[y + 1][x].south = WALL;
                }
            }
            break;
        case SOUTH:
            if (left_wall) {
                maze->m_walls[y][x].east = WALL;
                if (x < MAZE_HEIGHT - 1) {
                    maze->m_walls[y][x + 1].west = WALL;
                }
            }
            if (right_wall) {
                maze->m_walls[y][x].west = WALL;
                if (x > 0) {
                    maze->m_walls[y][x - 1].east = WALL;
                }
            }
            if (front_wall) {
                maze->m_walls[y][x].south = WALL;
                if (y > 0) {
                    maze->m_walls[y - 1][x].north = WALL;
                }
            }
            break;
        case EAST:
            if (left_wall) {
                maze->m_walls[y][x].north = WALL;
                if (y < MAZE_HEIGHT - 1) {
                    maze->m_walls[y + 1][x].south = WALL;
                }
            }
            if (right_wall) {
                maze->m_walls[y][x].south = WALL;
                if (y > 0) {
                    maze->m_walls[y - 1][x].north = WALL;
                }
            }
            if (front_wall) {
                maze->m_walls[y][x].east = WALL;
                if (x < MAZE_HEIGHT - 1) {
                    maze->m_walls[y][x + 1].west = WALL;
                }
            }
            break;
        case WEST:

            if (left_wall) {
                maze->m_walls[y][x].south = WALL;
                if (y > 0) {
                    maze->m_walls[y - 1][x].north = WALL;
                }
            }
            if (right_wall)
            {
                maze->m_walls[y][x].north = WALL;
                if (y < MAZE_HEIGHT - 1) {
                    maze->m_walls[y + 1][x].south = WALL;
                }
            }
            if (front_wall)
            {
                maze->m_walls[y][x].west = WALL;
                if (x > 0) {
                    maze->m_walls[y][x - 1].east = WALL;
                }
            }
    		break;

        default:
            break;
    }
}


#endif