#ifndef MOTION_H
#define MOTION_H

#include "profile.h"
#include "motors.h"
#include "maze.h"

const char *MOTION_TAG = "motion";

struct TurnParameters {
  int speed;         // mm/s    - constant forward speed during turn
  int entry_offset;  // mm      - distance from turn pivot to turn start
  int exit_offset;   // mm      - distance from turn pivot to turn end
  float angle;       // deg     - total turn angle
  float omega;       // deg/s   - maximum angular velocity
  float alpha;       // deg/s/s - angular acceleration
  int trigger;       //         - front sensor value at start of turn
};

const TurnParameters turn_params[4] = {
    //           speed, entry,   exit, angle, omega,  alpha, sensor threshold
    {SEARCH_TURN_SPEED,    15,     15, -90.f, 287.f, 2850.f, TURN_THRESHOLD}, // 0 => SS90EL
    {SEARCH_TURN_SPEED,    15,     15,  90.f, 287.f, 2850.f, TURN_THRESHOLD}, // 0 => SS90ER
    {SEARCH_TURN_SPEED,    0,     15, -45.f,  287.f, 2866.0, TURN_THRESHOLD}, // 0 => SS90L
    {SEARCH_TURN_SPEED,    0,     15,   45.f, 287.f, 2866.0, TURN_THRESHOLD}, // 0 => SS90R
};

enum TurnType {
    SS90EL = 0,
    SS90ER = 1,
    SS45EL = 2,
    SS45ER = 3,
    SS_FWD = 4
};

typedef struct {
  TurnType type;
  uint8_t num;
} MovementType;

MovementType* speed_movements = NULL;

void turn_IP(float angle, float omega, float alpha);

void turn_IP180() {
    static int direction = 1;
    set_target_velocity(&forward, 0.f);
    
    while (forward.velocity != 0) {
      vTaskDelay(1);
    }
    
    direction *= -1;  // alternate direction each 180
    steering_enabled = false;
    turn_IP(direction * 180, 250, 1500);
    steering_enabled = true;
    // maze.m_mouse_heading = behind_from(maze.m_mouse_heading);
}


void turn_IP(float angle, float omega, float alpha) {
    steering_enabled = false;
    Profile_Start(&rotation, angle, omega, 0.f, alpha);
    while (!Profile_Is_Finished(&rotation)) {
        vTaskDelay(1);
    }
    steering_enabled = true;
    return;
}


void move_forward(float distance, float target_vel, float end_vel, float acc) {
    steering_enabled = true;
    bool not_polled_next_cell_walls = true;
    Profile_Start(&forward, distance, target_vel, end_vel, acc);
    while (!Profile_Is_Finished(&forward)) {
        if (fabs(forward.final_position) - fabs(forward.position) <= SENSING_POS_OFFSET && not_polled_next_cell_walls)
        {
          not_polled_next_cell_walls = false;
          next_cell_front_wall = front_wall_present;
          next_cell_left_wall = left_wall_present;
          next_cell_right_wall = right_wall_present;
        }
        vTaskDelay(1);
    }
    return;
}


// stops in the center of the cell, steering is turned off and must be re-enabled
void stop_at_center() {
    // bool has_front_wall = front_wall_present;
    bool has_front_wall = cur_cell_front_wall;
    steering_enabled = false;

    float remaining = HALF_CELL; // - forward.position;

    if (fabs(forward.velocity) < 1) {
      forward.velocity = 50.f;
    }
    if (has_front_wall) {
      Profile_Start(&forward, HALF_CELL * 0.25f, forward.velocity, 80.f, SEARCH_ACCELERATION);
      while (front_sensor_sum < FRONT_REFERENCE) {
        vTaskDelay(1);
      }

    } else {
      steering_enabled = true;
      Profile_Start(&forward, HALF_CELL, forward.velocity, 0.f, SEARCH_ACCELERATION);
      while (!Profile_Is_Finished(&forward)) {
        vTaskDelay(1);
      };
    }
    // makes sure robot stops.
    Profile_Stop(&forward);
  }

// algins mouse with the front wall, make sure to re-enable steering as it is false upon exitting the function
void align_with_front_wall() {
    steering_enabled = true;
    steering_mode = SM_FRONT_BASED;

    int alignment_buffer = 30;

    // while (alignment_buffer) {
    //   if(abs(front_align_error) < 10 && abs(front_dist_error) < 20) {
    //     alignment_buffer--;
    //   } else {
    //     alignment_buffer = 30;
    //   }

      // if (left_wall_present) {
      //   gpio_set_level(RED_LED, 0);
      // } else {
      //   gpio_set_level(RED_LED, 1);
      // }

      // if (right_wall_present) {
      //   gpio_set_level(GREEN_LED, 0);
      // } else {
      //   gpio_set_level(GREEN_LED, 1);
      // }

      // if (front_wall_present) {
      //   gpio_set_level(BLUE_LED, 0);
      // } else {
      //   gpio_set_level(BLUE_LED, 1);
      // }

      // vTaskDelay(2);
      vTaskDelay(600);
    // }

    steering_enabled = false;
    steering_mode = SM_SIDE_BASED;
}

void turn_smooth(uint8_t turn_id) {
    steering_enabled = true;
    set_target_velocity(&forward, SEARCH_TURN_SPEED);
    TurnParameters params = turn_params[turn_id];

    float trigger = params.trigger;
    if (left_wall_present) {
      trigger += EXTRA_WALL_ADJUST;
    }
    if (right_wall_present) {
      trigger += EXTRA_WALL_ADJUST;
    }

    bool triggered_by_sensor = false;
    
    Profile_Start(&forward, params.entry_offset, SEARCH_TURN_SPEED, params.speed, SEARCH_ACCELERATION);
    while(!Profile_Is_Finished(&forward)) { 
      if (front_sensor_sum > SEARCH_TURN_TRIGGER) {
        triggered_by_sensor = true;
        set_target_velocity(&forward, forward.velocity);
        break;
      }

      vTaskDelay(1); 
    }

    steering_enabled = false;
    Profile_Start(&rotation, params.angle, params.omega, 0, params.alpha);
    
    while(!Profile_Is_Finished(&rotation)) { vTaskDelay(1); }

    // poll sensors for next cell's walls
    bool not_polled_next_cell_walls = true;
    steering_enabled = true;
    Profile_Start(&forward, params.exit_offset, SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, SEARCH_ACCELERATION);
    
    next_cell_front_wall = front_wall_present;
    next_cell_left_wall = left_wall_present;
    next_cell_right_wall = right_wall_present;
    
    while(!Profile_Is_Finished(&forward)) { 
      if (fabs(forward.final_position) - fabs(forward.position) <= SENSING_POS_OFFSET && not_polled_next_cell_walls)
      {
        not_polled_next_cell_walls = false;
      }
      vTaskDelay(1); 
    }    
  }

void turn_back() {
  bool has_front_wall = cur_cell_front_wall;
  bool has_left_wall = cur_cell_left_wall;
  bool has_right_wall = cur_cell_right_wall;
  static int8_t dir_turn = 1; // 1 is right, 0 is left

  stop_at_center();

  if (has_front_wall) {
    align_with_front_wall();
  }

  if (has_left_wall && has_right_wall) {
    if (dir_turn > 0) {
      turn_IP(90.f, 287.f, 2850.f);
      align_with_front_wall();
      turn_IP(90.f, 287.f, 2850.f);
    } else {
      turn_IP(-90.f, 287.f, 2850.f);
      align_with_front_wall();
      turn_IP(-90.f, 287.f, 2850.f);
    }
    dir_turn *= -1;
  } else if (has_left_wall) {
      turn_IP(-90.f, 287.f, 2850.f);
      align_with_front_wall();
      turn_IP(-90.f, 287.f, 2850.f);
  } else if (has_right_wall) {
      turn_IP(90.f, 287.f, 2850.f);
      align_with_front_wall();
      turn_IP(90.f, 287.f, 2850.f);
  } else {
    turn_IP180();
  }

  bool not_polled_next_cell_walls = true;
  Profile_Start(&forward, HALF_CELL, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
  while(!Profile_Is_Finished(&forward)) { 
    if (fabs(forward.final_position) - fabs(forward.position) <= SENSING_POS_OFFSET && not_polled_next_cell_walls)
      {
        not_polled_next_cell_walls = false;
      }
    next_cell_front_wall = front_wall_present;
    next_cell_left_wall = left_wall_present;
    next_cell_right_wall = right_wall_present;
    vTaskDelay(1); }
}

void search(Pos target_pos, bool hand_start)
{
  static uint8_t led_state = 0;
  maze.m_goal = target_pos;
  // scan_new_walls(&maze);
  // flood(&maze);
  float drive_offset;

  steering_enabled = false;
  
  // sets next_cell_wall values to those of the current cell
  next_cell_right_wall = !is_available(&maze, maze.m_mouse_pos, right90_from(maze.m_mouse_heading));
  next_cell_left_wall = !is_available(&maze, maze.m_mouse_pos, left90_from(maze.m_mouse_heading));
  next_cell_front_wall = !is_available(&maze, maze.m_mouse_pos, maze.m_mouse_heading);

  if (hand_start) {
    Profile_Start(&forward, -HALF_CELL, SEARCH_VELOCITY / 4, 0.f, SEARCH_ACCELERATION / 3);
    while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
    vTaskDelay(200);
    drive_offset = BACK_TO_CENTER_DIST;
    hand_start = false;
  } else {
    drive_offset = HALF_CELL;
  }

  steering_enabled = true;

  while (maze.m_mouse_pos.x != maze.m_goal.x || maze.m_mouse_pos.y != maze.m_goal.y)
  {
    scan_new_walls(&maze);
    flood(&maze);

    if (cur_cell_left_wall) {
      gpio_set_level(RED_LED, 0);
    } else {
      gpio_set_level(RED_LED, 1);
    }

    if (cur_cell_right_wall) {
      gpio_set_level(GREEN_LED, 0);
    } else {
      gpio_set_level(GREEN_LED, 1);
    }

    if (cur_cell_front_wall) {
      gpio_set_level(BLUE_LED, 0);
    } else {
      gpio_set_level(BLUE_LED, 1);
    }

    // ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

    Heading bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);
    uint8_t headingChange = ((bestHeading - maze.m_mouse_heading) >> 1) & 0x3;
    maze.m_mouse_heading = bestHeading;

    switch (headingChange) {
      case AHEAD:
        if (hand_start) {
          move_forward(FULL_CELL - drive_offset, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
          hand_start = false;
        } else {
          move_forward(FULL_CELL, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
        }
        break;

      case RIGHT:
        turn_smooth(SS90ER);
        break;

      case LEFT:
        turn_smooth(SS90EL);
        break;

      case BACK:
        turn_back();
        break;

      default:
        break;
    }

    maze.m_mouse_pos = neighbor(maze.m_mouse_pos, bestHeading);
    // ESP_LOGI(MOTION_TAG, "moved to x %d y %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y);
  }

  // ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

  // print_maze_state(&maze);
  ESP_LOGI(MOTION_TAG, "reached goal");

  stop_at_center();
}

void basic_search(Pos target_pos, bool hand_start)
{
  static uint8_t led_state = 0;
  maze.m_goal = target_pos;
  // scan_new_walls(&maze);
  // flood(&maze);
  float drive_offset;

  steering_enabled = false;
  
  // sets next_cell_wall values to those of the current cell
  next_cell_right_wall = !is_available(&maze, maze.m_mouse_pos, right90_from(maze.m_mouse_heading));
  next_cell_left_wall = !is_available(&maze, maze.m_mouse_pos, left90_from(maze.m_mouse_heading));
  next_cell_front_wall = !is_available(&maze, maze.m_mouse_pos, maze.m_mouse_heading);

  if (hand_start) {
    Profile_Start(&forward, -HALF_CELL, SEARCH_VELOCITY / 4, 0.f, SEARCH_ACCELERATION / 3);
    while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
    vTaskDelay(200);
    drive_offset = BACK_TO_CENTER_DIST;
    hand_start = false;
  } else {
    drive_offset = HALF_CELL;
  }

  steering_enabled = true;

  while (maze.m_mouse_pos.x != maze.m_goal.x || maze.m_mouse_pos.y != maze.m_goal.y)
  {
    scan_new_walls(&maze);
    flood(&maze);

    // if (cur_cell_left_wall) {
    //   gpio_set_level(RED_LED, 0);
    // } else {
    //   gpio_set_level(RED_LED, 1);
    // }

    // if (cur_cell_right_wall) {
    //   gpio_set_level(GREEN_LED, 0);
    // } else {
    //   gpio_set_level(GREEN_LED, 1);
    // }

    // if (cur_cell_front_wall) {
    //   gpio_set_level(BLUE_LED, 0);
    // } else {
    //   gpio_set_level(BLUE_LED, 1);
    // }

    // ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

    Heading bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);
    uint8_t headingChange = ((bestHeading - maze.m_mouse_heading) >> 1) & 0x3;
    maze.m_mouse_heading = bestHeading;

    switch (headingChange) {
      case AHEAD:
        if (hand_start) {
          move_forward(FULL_CELL - drive_offset, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
          hand_start = false;
        } else {
          move_forward(FULL_CELL, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
        }
        break;

      case RIGHT:
        move_forward(HALF_CELL, SEARCH_VELOCITY, 0.f, SEARCH_ACCELERATION);

        if (!is_available(&maze, maze.m_mouse_pos, maze.m_mouse_heading)) {
          align_with_front_wall();
        }
        turn_IP(90.f, 287.f, 2850.f);
        break;

      case LEFT:
        move_forward(HALF_CELL, SEARCH_VELOCITY, 0.f, SEARCH_ACCELERATION);
          
        if (!is_available(&maze, maze.m_mouse_pos, maze.m_mouse_heading)) {
          align_with_front_wall();
        }
        turn_IP(-90.f, 287.f, 2850.f);
        move_forward(HALF_CELL, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
        break;

      case BACK:
        turn_back();
        break;

      default:
        break;
    }

    maze.m_mouse_pos = neighbor(maze.m_mouse_pos, bestHeading);
    // ESP_LOGI(MOTION_TAG, "moved to x %d y %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y);
  }

  // ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

  // print_maze_state(&maze);
  ESP_LOGI(MOTION_TAG, "reached goal");

  stop_at_center();
}

int generate_speed_moves(Pos target_pos) {
  maze.m_goal = target_pos;
  // change maze mask for speed run 
  maze.m_mask = MASK_CLOSED;
  flood(&maze);

  speed_movements = (MovementType*)malloc(sizeof(MovementType) * maze.m_cost[0][0]);

  Pos virtual_pos = (Pos){0, 0};
  Heading virtual_heading = NORTH;

  speed_movements[0].type = SS_FWD;
  speed_movements[0].num = 1;
  uint8_t idx = 0;

  while (maze.m_cost[virtual_pos.y][virtual_pos.x] != 0) 
  {
    Heading best_heading = heading_to_smallest(&maze, virtual_pos, virtual_heading);
    uint8_t headingChange = ((best_heading - maze.m_mouse_heading) >> 1) & 0x3;
    maze.m_mouse_heading = best_heading;

    switch(headingChange) 
    {
      case AHEAD:
        if (speed_movements[idx].type == SS_FWD) {
          speed_movements[idx].num++;
        } else {
          idx++;
          speed_movements[idx].type = SS_FWD;
          speed_movements[idx].num++;
        }
        break;

      case RIGHT:
        if (speed_movements[idx].type == SS90ER) {
          speed_movements[idx].num++;
        } else {
          idx++;
          speed_movements[idx].type = SS90ER;
          speed_movements[idx].num++;
        }
        break;

      case LEFT:
        if (speed_movements[idx].type == SS90EL) {
          speed_movements[idx].num++;
        } else {
          idx++;
          speed_movements[idx].type = SS90EL;
          speed_movements[idx].num++;
        }
        break;

      default:
        break;
    }
  }

  speed_movements = (MovementType*)realloc((void*)speed_movements, sizeof(MovementType) * (idx + 1));
  return idx + 1;
}


void destroy_speed_moves(MovementType* speed_moves)
{
  free(speed_moves);
}


#endif