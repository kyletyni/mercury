#ifndef PROFILE_H
#define PROFILE_H

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// time in ms between loop calls, could be passed as variable in ms
#define LOOP_INTERVAL  (1.f / 500.f)

typedef enum : uint8_t {
	PS_IDLE = 0,
	PS_ACCELERATING,
	PS_BRAKING,
	PS_DONE
} ProfileState;

typedef struct {
	ProfileState state;
	int8_t sign;

	// units are use millimeters to seconds
	float position;
	float speed;
	float acceleration;
	float one_over_acc;
	float target_speed;

	// speed and position at the end a movement
	float final_speed;
	float final_position;
} Profile;


void Profile_Reset(Profile* profile)
{
	profile->state = PS_IDLE;
	profile->position = 0;
	profile->speed = 0;
	profile->target_speed = 0;
}


void Profile_Start(Profile* profile, float distance, float target_speed, float final_speed, float acceleration) {
    profile->sign = (distance < 0) ? -1 : +1;

    if (distance < 0) {
        distance = -distance;
    }
    
    if (distance < 1.0) {
        profile->state = PS_DONE;
        return;
    }
    
    if (final_speed > target_speed) {
        final_speed = target_speed;
    }

    // profile->position = 0;
    profile->final_position = distance;
    profile->target_speed = m_sign * fabsf(target_speed);
    profile->final_speed = m_sign * fabsf(final_speed);
    profile->acceleration = fabsf(acceleration);
    
    if (profile->acceleration >= 1) {
        profile->one_over_acc = 1.0f / profile->acceleration;
    } else {
        profile->one_over_acc = 1.0;
    }
    
    profile->state = PS_ACCELERATING;
}


void  Profile_Update(Profile* profile) {
    if(profile->state == PS_IDLE) {
		return;
	}

	float delta_v = profile->acceleration * LOOP_INTERVAL;
	float remaining_dist = fabs(profile->final_position) - fabs(profile->position);

	if(profile->state == PS_ACCELERATING) {
		if(remaining_dist < Profile_Get_Braking_Dist(profile)) {
			profile->state = PS_BRAKING;

			if (profile->final_speed == 0) {
				profile->target_speed = profile->sign * 5.0f;
			}
			else {
				profile->target_speed = profile->final_speed;
			};
		}
	}

    // reaching the target speed
	if (profile->speed < profile->target_speed)	{
		profile->speed += delta_v;

		if (profile->speed > profile->target_speed)	{
			profile->speed = profile->target_speed;
		}
	}

	if (profile->speed > profile->target_speed)	{
		profile->speed -= delta_v;

		if (profile->speed < profile->target_speed)	{
			profile->speed = profile->target_speed;
		}
	}

    // update positoin
	profile->position += profile->speed * LOOP_INTERVAL;

	if (profile->state != PS_DONE && remaining_dist < 0.1f)
	{
		profile->state = PS_DONE;
		profile->target_speed = profile->final_speed;
	}

}


void  Profile_Stop(Profile* profile) {   
    profile->target_speed = 0;
}

void  Profile_Finish(Profile* profile) {
    profile->speed = profile->target_speed;
    profile->state = PS_DONE;
}

bool  Profile_Is_Finished(Profile* profile) {
    return profile->state == PS_FINISHED;
}

float Profile_Get_Braking_Dist(Profile* profile){
    return fabsf(profile->speed * profile->speed - profile->final_speed * profile->final_speed) * 0.5 * profile->one_over_acc;
}


#endif