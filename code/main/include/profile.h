#ifndef PROFILE_H
#define PROFILE_H

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"


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
	float velocity;
	float acceleration;
	float one_over_acc;
	float target_velocity;

	// velocity and position at the end a movement
	float final_velocity;
	float final_position;
} Profile;

Profile forward;
Profile rotation;

float Profile_Get_Braking_Dist(Profile* profile);

void Profile_Reset(Profile* profile)
{
	profile->state = PS_IDLE;
	profile->position = 0.f;
	profile->velocity = 0.f;
	profile->target_velocity = 0.f;
}


void Profile_Start(Profile* profile, float distance, float target_velocity, float final_velocity, float acceleration) {
    profile->sign = (distance < 0) ? -1 : +1;

    if (distance < 0) {
        distance = -distance;
    }
    
    if (distance < 1.0) {
        profile->state = PS_DONE;
        return;
    }
    
    if (final_velocity > target_velocity) {
        final_velocity = target_velocity;
    }

	// want to subtract distance of cell here instead of setting it to zero on every new start
    profile->position = 0;
    profile->final_position = distance;
    profile->target_velocity = profile->sign * fabsf(target_velocity);
    profile->final_velocity = profile->sign * fabsf(final_velocity);
    profile->acceleration = fabsf(acceleration);
    
    if (profile->acceleration >= 1) {
        profile->one_over_acc = 1.0f / profile->acceleration;
    } else {
        profile->one_over_acc = 1.0;
    }
    
    profile->state = PS_ACCELERATING;
}


void Profile_Update(Profile* profile) {
    if(profile->state == PS_IDLE) {
		return;
	}

	float delta_v = profile->acceleration * LOOP_INTERVAL;
	float remaining_dist = fabs(profile->final_position) - fabs(profile->position);

	if(profile->state == PS_ACCELERATING) {
		if(remaining_dist < Profile_Get_Braking_Dist(profile)) {
			profile->state = PS_BRAKING;

			if (profile->final_velocity == 0) {
				profile->target_velocity = profile->sign * 5.0f;
			}
			else {
				profile->target_velocity = profile->final_velocity;
			};
		}
	}

    // reaching the target speed
	if (profile->velocity < profile->target_velocity)	{
		profile->velocity += delta_v;

		if (profile->velocity > profile->target_velocity)	{
			profile->velocity = profile->target_velocity;
		}
	}

	if (profile->velocity > profile->target_velocity)	{
		profile->velocity -= delta_v;

		if (profile->velocity < profile->target_velocity)	{
			profile->velocity = profile->target_velocity;
		}
	}

    // update position
	profile->position += profile->velocity * LOOP_INTERVAL;

	if (profile->state != PS_DONE && remaining_dist < 0.2f)
	{
		profile->state = PS_DONE;
		profile->target_velocity = profile->final_velocity;
	}
}


void Profile_Stop(Profile* profile) {   
    profile->target_velocity = 0;
}

void Profile_Finish(Profile* profile) 
{
    profile->velocity = profile->target_velocity;
    profile->state = PS_DONE;
}

bool Profile_Is_Finished(Profile* profile) 
{
    return profile->state == PS_DONE;
}

float Profile_Get_Braking_Dist(Profile* profile) 
{
    return fabsf(profile->velocity * profile->velocity - profile->final_velocity * profile->final_velocity) * 0.5 * profile->one_over_acc;
}

void set_target_velocity(Profile* profile, float vel) 
{
	profile->target_velocity = vel;
}


#endif