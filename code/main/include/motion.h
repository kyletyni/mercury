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
    {SEARCH_TURN_SPEED,    70,     80, -90.0, 287.0, 2866.0, TURN_THRESHOLD}, // 0 => SS90L
    {SEARCH_TURN_SPEED,    70,     80,  90.0, 287.0, 2866.0, TURN_THRESHOLD}, // 0 => SS90R
};

enum TurnType {
    SS90EL = 0,
    SS90ER = 1,
    SS45EL = 2,
    SS45ER = 3,
};

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
    Profile_Start(&rotation, angle, omega, 0.f, alpha);
    while (!Profile_Is_Finished(&rotation)) {
        vTaskDelay(1);
    }
    return;
}


void move_forward(float distance, float target_vel, float end_vel, float acc) {
    steering_enabled = true;
    Profile_Start(&forward, distance, target_vel, end_vel, acc);
    while (!Profile_Is_Finished(&forward)) {
        vTaskDelay(1);
    }
    return;
}

// stops in the center of the cell, steering is turned off and must be re-enabled
void stop_at_center() {
    bool has_front_wall = front_wall_present;
    steering_enabled = false;

    float remaining = HALF_CELL; // - forward.position;

    if (has_front_wall) {
      Profile_Start(&forward, HALF_CELL * 0.25f, forward.velocity, 80.f, SEARCH_ACCELERATION);
      while (front_sensor_sum < FRONT_REFERENCE) {
        vTaskDelay(1);
      }
    } else {
      Profile_Start(&forward, HALF_CELL, forward.velocity, 0.f, SEARCH_ACCELERATION);
      while (!Profile_Is_Finished(&forward)) {
        vTaskDelay(1);
      };
    }
    // makes sure robot stops.
    Profile_Stop(&forward);
  }


void turn_smooth(uint8_t turn_id) {
    steering_enabled = false;
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
    //float turn_point = FULL_CELL + HALF_CELL - params.entry_offset;
    float turn_point = params.entry_offset;
    // while (motion.position() < turn_point) {
    //   if (sensors.get_front_sum() > trigger) {
        // motion.set_target_velocity(motion.velocity());
        // triggered_by_sensor = true;
        // break;
    //   }
    // }

    // char note = triggered_by_sensor ? 's' : 'd';
    // char dir = (turn_id & 1) ? 'R' : 'L';
    // reporter.log_action_status(dir, note, m_location, m_heading);  // the sensors triggered the turn
    // finally we get to actually turn
    
    Profile_Start(&forward, params.entry_offset, SEARCH_TURN_SPEED, params.speed, SEARCH_ACCELERATION);
    while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }

    // Profile_Start(&forward, 150.f, params.speed, params.speed, SEARCH_ACCELERATION);
    Profile_Start(&rotation, params.angle, params.omega, 0, params.alpha);
    
    while(!Profile_Is_Finished(&rotation)) { vTaskDelay(1); }
    // rotation.position = 0.f;
    // steering_enabled = true;
    Profile_Start(&forward, params.exit_offset, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
    while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
    // robot should be at output offset - run to the sensing position
    int end_point = HALF_CELL + params.exit_offset;
    // Profile_Start(&forward, 150.f, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
    // Profile_Start(SENSING_POSITION - end_point, motion.velocity(), SEARCH_VELOCITY, SEARCH_ACCELERATION);
    // motion.set_position(SENSING_POSITION);
  }

void turn_back() {
  stop_at_center();
  turn_IP180();
  Profile_Start(&forward, HALF_CELL, SEARCH_VELOCITY, 0, SEARCH_ACCELERATION);
  while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
}

void search(Pos target_pos, bool hand_start)
{
  static uint8_t led_state = 0;
  maze.m_goal = target_pos;
  // scan_new_walls(&maze);
  // flood(&maze);
  float drive_offset;

  steering_enabled = false;
  if (hand_start) {
    Profile_Start(&forward, -BACK_TO_CENTER_DIST, SEARCH_VELOCITY / 4, 0.f, SEARCH_ACCELERATION / 3);
    while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
    vTaskDelay(200);
    drive_offset = BACK_TO_CENTER_DIST;
  } else {
    drive_offset = HALF_CELL;
  }

  steering_enabled = true;

  while (maze.m_mouse_pos.x != maze.m_goal.x || maze.m_mouse_pos.y != maze.m_goal.y)
  {
    // vTaskDelay(500);
    // Profile_Stop(&forward);
    // Profile_Stop(&rotation);
    // print_maze_state(&maze);
    // while (1) {
    //     if (gpio_get_level(BTN2) == 0) {
    //         // Optional: Debounce the button press to avoid multiple triggers
    //         vTaskDelay(200 / portTICK_PERIOD_MS); // Delay for 100ms
    //         break;
    //     }
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    // gpio_set_level(BLUE_LED, led_state);
    // led_state = ~led_state;
    scan_new_walls(&maze);
    flood(&maze);

    if (left_wall_present) {
      gpio_set_level(RED_LED, 0);
    } else {
      gpio_set_level(RED_LED, 1);
    }

    if (right_wall_present) {
      gpio_set_level(GREEN_LED, 0);
    } else {
      gpio_set_level(GREEN_LED, 1);
    }

    if (front_wall_present) {
      gpio_set_level(BLUE_LED, 0);
    } else {
      gpio_set_level(BLUE_LED, 1);
    }

    // ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

    Heading bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);
    uint8_t headingChange = ((bestHeading - maze.m_mouse_heading) >> 1) & 0x3;

    if (maze.m_mouse_pos.x > MAZE_WIDTH || maze.m_mouse_pos.y > MAZE_HEIGHT) {
      gpio_set_level(RED_LED, 0);
    }

    switch (headingChange) {
      case AHEAD:
        if (drive_offset > 0) {
          move_forward(FULL_CELL - drive_offset, SEARCH_VELOCITY, SEARCH_VELOCITY, SEARCH_ACCELERATION);
          drive_offset = 0;
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
        // gpio_set_level(RED_LED, 0);
        break;
    }

    
    maze.m_mouse_pos = neighbor(maze.m_mouse_pos, bestHeading);
    maze.m_mouse_heading = bestHeading;
    ESP_LOGI(MOTION_TAG, "moved to x %d y %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y);
  }

  ESP_LOGI(MOTION_TAG, "x %d y %d head %d", maze.m_mouse_pos.x, maze.m_mouse_pos.y, maze.m_mouse_heading);

  // print_maze_state(&maze);
  // ESP_LOGI(MOTION_TAG, "reached goal");

  stop_at_center();
}


#endif