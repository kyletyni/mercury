#ifndef GYRO_H
#define GYRO_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/message_buffer.h>

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "encoder.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

static const char *GYRO_TAG = "gyro";

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

typedef struct {
	uint16_t port;
	char ipv4[20]; // xxx.xxx.xxx.xxx
} PARAMETER_t;


typedef struct {
	float quatx;
	float quaty;
	float quatz;
	float quatw;
	float roll;
	float pitch;
	float yaw;
} POSE_t;

float mouse_yaw;
float mouse_yaw_capture;
float yaw_change = 0;

bool gyro_init = false;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if 0
	float _roll = ypr[2] * RAD_TO_DEG;
	float _pitch = ypr[1] * RAD_TO_DEG;
	float _yaw = ypr[0] * RAD_TO_DEG;
	ESP_LOGI(GYRO_TAG, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
#endif
	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	//ESP_LOGI(GYRO_TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

void mpu6500(void *pvParameters) {
    // init gyro
    mpu.initialize();
	float yaw;
	float prev_yaw;
	bool first_time = true;

	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
	I2Cdev::readByte(0x68, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(GYRO_TAG, "getDeviceID=0x%x", buffer[0]);

    // Initialize DMP
	devStatus = mpu.dmpInitialize();
	ESP_LOGI(GYRO_TAG, "devStatus=%d", devStatus);
	if (devStatus != 0) {
		ESP_LOGE(GYRO_TAG, "DMP Initialization failed [%d]", devStatus);
		while(1) {
			vTaskDelay(1);
		}
	}

    // This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-2610);
	mpu.setYAccelOffset(5861);
	mpu.setZAccelOffset(10281);
	mpu.setXGyroOffset(198);
	mpu.setYGyroOffset(65);
	mpu.setZGyroOffset(-18);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.setDMPEnabled(true);

	gyro_init = true;
	ESP_LOGI(GYRO_TAG, "DMP Initialization successfully");

	TickType_t xFrequency = pdMS_TO_TICKS(4); // convert to ms
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
	while(1) {
	    xLastWakeTime = xTaskGetTickCount();

		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
			getYawPitchRoll();
			yaw = ypr[0] * RAD_TO_DEG;

			if (first_time) {
				first_time = false;
				prev_yaw = yaw;
			}

			yaw_change = yaw - prev_yaw;

			mouse_yaw = yaw;
			// mouse_yaw += yaw_change;
			prev_yaw = yaw;
			// float _roll = ypr[2] * RAD_TO_DEG;
			// float _pitch = ypr[1] * RAD_TO_DEG;

			//getQuaternion();
			// getEuler();
			//getRealAccel();
			//getWorldAccel();

			vTaskDelayUntil(&xLastWakeTime, xFrequency);
			TickType_t xElapsedTime = (xTaskGetTickCount() - xLastWakeTime);
			if (xElapsedTime > xFrequency) {
				ESP_LOGW(GYRO_TAG, "WARNING: 'gyroTask' exceeded deadline: %ld ms!", xElapsedTime);
			}
		}
    }

    vTaskDelete(NULL);
}

#endif