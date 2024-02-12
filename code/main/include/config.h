#ifndef CONFIG_H
#define CONFIG_H

#define PI_VAL 3.1415f
#define MERCURY_CFG

#ifdef MERCURY_CFG

    #define ROTATION_BIAS 0
    #define WHEEL_DIAMETER 22.6f
    #define ENCODER_COUNTS_PER_REV 4096
    #define GEAR_RATIO (38.f / 22.f)//(22.f / 38.f)
    #define FF_VOLTAGE 0.00194f
    #define BIAS_VOLTAGE 0.45f // might have to set higher


    #define FWD_TIME_CONST 0.2f
    #define ROT_TM 0.2f
    #define FF_ACC_VOLTAGE (FWD_TIME_CONST * FF_VOLTAGE)

    #define MOUSE_RADIUS 40.5f

    #define FWD_KP 0.5f
    #define FWD_KD 0.25f
    // #define ROT_KP 0.08f
    // #define ROT_KD 0.04f

    #define LOOP_FREQUENCY (500.f)

    // time in ms between loop calls, could be passed as variable in ms
    #define LOOP_INTERVAL  (1.f / LOOP_FREQUENCY)
#endif

const float ROT_KM = 347.f;  // deg/s/Volt
const float ROT_ZETA = 0.707;   
const float ROT_TD = ROT_TM;

const float ROT_KP = 0.3f; //16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = 0.12f; //LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

const float MM_PER_ENC_COUNT = PI_VAL * WHEEL_DIAMETER / (ENCODER_COUNTS_PER_REV * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_ENC_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_ENC_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.f / (2.f * MOUSE_RADIUS * PI_VAL));
const float RADIANS_PER_DEGREE = 2.f * PI_VAL / 360.f;
const float DEGREES_PER_RADIAN = 360.f / 2.f * PI_VAL;

#endif