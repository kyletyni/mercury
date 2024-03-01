#ifndef CONFIG_H
#define CONFIG_H

#define PI_VAL 3.1415f
#define MERCURY_CFG

#define APEC_CONFIG

#ifdef MERCURY_CFG

    #define ROTATION_BIAS 0
    #define WHEEL_DIAMETER 22.8f //22.6f
    #define ENCODER_COUNTS_PER_REV 4096
    #define GEAR_RATIO (38.f / 22.f)
    #define FF_VOLTAGE 0.00193f
    #define BIAS_VOLTAGE 0.47f // might have to set higher

    #define FWD_TIME_CONST 0.15f
    #define ROT_TM 0.1f
    #define FF_ACC_VOLTAGE (FWD_TIME_CONST * FF_VOLTAGE)

    #define MOUSE_RADIUS 41.5 // 40.5f

    #define FWD_KP 0.25f
    #define FWD_KD 0.02f//0.15f

    #define STEERING_KP 0.3f //0.4f //0.8f
    #define STEERING_KD 0.05f //0.35f

    #define FRONT_STEERING_KP 0.15f

    #define STEERING_ADJUSTMENT_LIMIT 10.f


    #define LOOP_FREQUENCY (500.f)
    #define LOOP_INTERVAL  (1.f / LOOP_FREQUENCY)

    #ifdef APEC_CONFIG
        // RAW values
        #define DIAG_RIGHT_CALIB  230 // 220 
        #define DIAG_LEFT_CALIB   260
        #define FRONT_RIGHT_CALIB 520
        #define FRONT_LEFT_CALIB  370

        // nominal vals
        #define FRONT_LEFT_CALIB_WHEN_CENTERED  620
        #define FRONT_RIGHT_CALIB_WHEN_CENTERED 525
        // front sensor sum to trigger a turn, because the front wall is close enough
        
        // translated to nominal (front sensor sum)
        #define SEARCH_TURN_TRIGGER 300

        #define RIGHT_THRESHOLD 67 // 82 //66 // 72 // 65
        #define LEFT_THRESHOLD  63 //65 // 72 // 65

        #define RIGHT_THRESHOLD_W_FRONT_WALL 85
        #define LEFT_THRESHOLD_W_FRONT_WALL 90

        #define FRONT_THRESHOLD 180 // 115 //135 // 170
        #define FRONT_THRESHOLD_TO_DSIABLE_SIDE_CORRECTION 220
        #define FRONT_THRESHOLD_FOR_HIGHER_SIDE_DETECT 220//175

        // above this value for the front sensor sum, the side sensors become
        // unreliable to do steering
        #define SIDE_RELIABILITY_THRESHOLD 270 
        const float FRONT_REFERENCE = 1200; // 1000;
    #endif

    #ifndef APEC_CONFIG
        #define DIAG_RIGHT_CALIB  295
        #define DIAG_LEFT_CALIB   280
        #define FRONT_RIGHT_CALIB 584
        #define FRONT_LEFT_CALIB  532
        #define FRONT_LEFT_CALIB_WHEN_CENTERED  690
        #define FRONT_RIGHT_CALIB_WHEN_CENTERED 595
        // front sensor sum to trigger a turn, because the front wall is close enough
        #define SEARCH_TURN_TRIGGER 258
    
        #define RIGHT_THRESHOLD 66 // 72 // 65
        #define LEFT_THRESHOLD  66 // 72 // 65
        #define FRONT_THRESHOLD 150 // 170
        
        // front sensor sum when the mouse is centered in the cell
        // its the converted value reading
        const float FRONT_REFERENCE = 900;
    #endif

    #define SIDE_NOMINAL  100.f
    #define FRONT_NOMINAL 200.f


    #define TURN_THRESHOLD 4
    #define EXTRA_WALL_ADJUST 5

    #define FULL_CELL 180.f
    #define HALF_CELL FULL_CELL / 2.f

    #define BACK_TO_CENTER_DIST 37 //53

    #define WALL_TO_ABSENT_VAL 43

    // FULL_CELL - 15mm
    #define SENSING_POS_OFFSET 15.f
    #define SENSING_POS FULL_CELL - SENSING_POS_OFFSET

#endif

const uint16_t SMOOTH_TURN_SPEED = 500;

const float ROT_KM = 347.f;  // deg/s/Volt
const float ROT_ZETA = 0.707;   
const float ROT_TD = ROT_TM;

const float ROT_KP = 0.15f; //0.2f; //16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = 0.04f; //LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

const float MM_PER_ENC_COUNT = PI_VAL * WHEEL_DIAMETER / (ENCODER_COUNTS_PER_REV * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_ENC_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_ENC_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.f / (2.f * MOUSE_RADIUS * PI_VAL));
const float RADIANS_PER_DEGREE = 2.f * PI_VAL / 360.f;
const float DEGREES_PER_RADIAN = 360.f / 2.f * PI_VAL;

const float FRONT_LEFT_SCALE = FRONT_NOMINAL / FRONT_LEFT_CALIB;
const float FRONT_RIGHT_SCALE = FRONT_NOMINAL / FRONT_RIGHT_CALIB;
const float DIAG_LEFT_SCALE = SIDE_NOMINAL / DIAG_LEFT_CALIB;
const float DIAG_RIGHT_SCALE = SIDE_NOMINAL / DIAG_RIGHT_CALIB;


const uint16_t SEARCH_VELOCITY = 400;
const uint16_t SEARCH_TURN_VELOCITY = 300;
const uint16_t SEARCH_ACCELERATION = 2000;
const uint16_t SEARCH_TURN_SPEED = 300;

const uint16_t SPEED_VELOCITY = 400;
const uint16_t SPEED_ACCELERATION = 2000;

#endif