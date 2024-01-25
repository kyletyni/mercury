#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

#define MAZE_SIZE 16
#define MAZE_HEIGHT 16
#define MAZE_WIDTH 16
#define MAX_COST (MAZE_HEIGHT * MAZE_WIDTH - 1)

#define NORTH_WALL 0b00001000
#define EAST_WALL  0b00000100
#define SOUTH_WALL 0b00000010
#define WEST_WALL  0b00000001


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
  return (Heading)((heading + 4) % HEADING_COUNT);
}

typedef enum {
    ABSENT = 0,
    PRESENT = 1,
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
  MASK_OPEN = 0x01,    // open maze for search
  MASK_CLOSED = 0x03,  // closed maze for fast run
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
            return (Pos){-1, -1};
            break;
    }
    if (is_in_maze(neighbor_pos))
        return neighbor_pos;
    else 
        return (Pos){-1, -1};
}

typedef struct {
	uint8_t m_cost[MAZE_SIZE][MAZE_SIZE];
	WallInfo m_walls[MAZE_SIZE][MAZE_SIZE];

	Pos m_mouse_pos;
	Heading m_mouse_heading;

	Pos* m_goals;
	uint8_t m_num_goals;
} Maze;


// Maze Functions
void initializeMaze(Maze* maze, Pos start);
void destroyMaze(Maze* maze);

bool is_absent(Maze* maze, Pos cell, Heading heading) {
    bool result = false;
    WallInfo walls = maze->m_walls[cell.x][cell.y];
    switch (heading) {
      case NORTH:
        result = (walls.north) == ABSENT;
        break;
      case EAST:
        result = (walls.east) == ABSENT;
        break;
      case SOUTH:
        result = (walls.south) == ABSENT;
        break;
      case WEST:
        result = (walls.west) == ABSENT;
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
			maze->m_cost[i][j] = MAX_COST;
		}
	}

    for (uint8_t i = 0; i < maze->m_num_goals; i++) {
		maze->m_cost[maze->m_goals[i].y][maze->m_goals[i].x] = 0;
	}

	uint8_t head = 0;
	uint8_t tail = 0;
	Pos queue[MAX_COST / 2];
    
	queue[tail] = maze->m_goals[0];
	tail++;

    while (tail - head > 0)	{
		Pos here = queue[head];
        uint16_t newCost = maze->m_cost[here.x][here.y] + 1;

        for (uint8_t h = NORTH; h < HEADING_COUNT; h += 2) {
            Heading heading = (Heading)(h);
            if (is_absent(maze, here, heading)) {
                Pos nextCell = neighbor(here, heading);
                if (maze->m_cost[nextCell.x][nextCell.y] > newCost) {
                    maze->m_cost[nextCell.x][nextCell.y] = newCost;
                    queue[tail] = nextCell;
                    tail++;
                }
            }
        }
    }
}

/// @brief returns the heading with the smallest neighboring cost value 
Heading heading_to_smallest(Maze* maze, Pos pos, Heading start_heading)
{
    Heading next_heading = start_heading;
    Heading best_heading = BLOCKED;
    uint8_t best_cost = maze->m_cost[pos.y][pos.x];
    uint8_t cost = MAX_COST;
    Pos next_cell;

    if (is_absent(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        cost = maze->m_cost[next_cell.y][next_cell.x];
        if (is_in_maze(next_cell) && cost < best_cost) {
            best_cost = cost;
            best_heading = next_heading;
        }
    }

    next_heading = right90_from(start_heading);
    if (is_absent(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        cost = maze->m_cost[next_cell.y][next_cell.x];
        if (is_in_maze(next_cell) && cost < best_cost) {
            best_cost = cost;
            best_heading = next_heading;
        }
    }

    next_heading = left90_from(start_heading);
    if (is_absent(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        cost = maze->m_cost[next_cell.y][next_cell.x];
        if (is_in_maze(next_cell) && cost < best_cost) {
            best_cost = cost;
            best_heading = next_heading;
        }
    }

    next_heading = behind_from(start_heading);
    if (is_absent(maze, pos, next_heading)) {
        next_cell = neighbor(pos, next_heading);
        cost = maze->m_cost[next_cell.y][next_cell.x];
        if (is_in_maze(next_cell) && cost < best_cost) {
            best_cost = cost;
            best_heading = next_heading;
        }
    }

    return best_heading;
}

/// @brief returns true if any walls in a cell have not been seen
bool has_unknown_walls(Maze* maze, Pos cell) {
    WallInfo walls_here = maze->m_walls[cell.x][cell.y];
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


#endif