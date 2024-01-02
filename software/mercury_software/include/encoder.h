#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDRESS 0x36


void init_position_sensor();
int read_encoder();
void update_current_position(int microsteps);

#endif