#ifndef DATA_DISPATCHER_H
#define DATA_DISPATCHER_H

#include <madgwick.h>
#include "stm32f1xx_hal.h"
#include "icm20948_hal.h"

typedef struct {
	// доп. структуры для хранения данных полученных при помощи драйвера
	AxisDataRaw_t raw_accel;
	AxisDataRaw_t raw_gyro;
	AxisDataScaled_t scaled_accel;
	AxisDataScaled_t scaled_gyro;

	quat_t old_quat;
	quat_t current_quat;
	quat_t d_quat;

	// текущая ориентация исходя из imu и фильтра
	float current_roll;
	float current_pitch;
	float current_yaw;

	// ориентация целевая, от lora
	float throttle;
	float control_roll;
	float control_pitch;
	float control_yaw;

	// выходы PID
	float pid_roll;
	float pid_pitch;
	float pid_yaw;

	// выходы микшера
	float motor1, motor2, motor3, motor4;

} fc_state_t;

#endif
