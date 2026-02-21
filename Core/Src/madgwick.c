#include "madgwick.h"
// То что static тебя ебать не должно, в main.c используются только не static
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define BETA_HIGH 0.01f
#define BETA_LOW 0.1f
uint32_t get_time_us(void){
	return HAL_GetTick() * 1000;
}

static float calc_3D_norm(const axis_scaled_t *vector){
	float norm = 1.0f;
	norm = sqrtf(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
	return norm;
}

static float calc_4D_norm(const quat_t *vector){
	float norm = 1.0f;
	norm = sqrtf(vector->w * vector->w + vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
	return norm;
}

static quat_t normalize_quat(const quat_t *current_q){
	quat_t q;
	float norm_q = 0;
	norm_q = calc_4D_norm(current_q);
	if (norm_q < 1e-6f){
		q.w = 1.0f;
		q.x = q.y = q.z = 0.0f;
		return q;
	}
	float inv_norm_q = 1.0f / norm_q;
	q.w = current_q->w * inv_norm_q;
	q.x = current_q->x * inv_norm_q;
	q.y = current_q->y * inv_norm_q;
	q.z = current_q->z * inv_norm_q;
	return q;
}

static float calc_beta_ratio(const axis_scaled_t *accels, float dt){
	float reference_beta = 1.0f;
	float accel_vector = sqrtf((accels->x)*(accels->x) + (accels->y)*(accels->y) + (accels->z)*(accels->z));
	if (accel_vector > 1.2f || accel_vector < 0.8f){reference_beta = BETA_LOW;}
	else {reference_beta = BETA_HIGH;}
	return reference_beta * (dt / 0.005f);
}

static axis_scaled_t calc_expected_accel_vector(const quat_t *q){
	axis_scaled_t expected_accel;
	expected_accel.x = 2 * (q->x * q->w - q->z * q->y);
	expected_accel.y = 2 * (q->y * q->w + q->z * q->x);
	expected_accel.z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
	float norm = calc_3D_norm(&expected_accel);
	if (norm > 1e-6f){
	float inv_norm = 1.0f/norm;
	expected_accel.x *= inv_norm;
	expected_accel.y *= inv_norm;
	expected_accel.z *= inv_norm;
	}
	else {
	expected_accel.x = 0.0f;
	expected_accel.y = 0.0f;
	expected_accel.z = 1.0f;
	}

	return expected_accel; //
}

static axis_scaled_t calc_accel_vector_error(const axis_scaled_t *current_accel, const axis_scaled_t *expected_accel){
	axis_scaled_t error;
	error.x = current_accel->y * expected_accel->z - current_accel->z * expected_accel->y;
	error.y = current_accel->z * expected_accel->x - current_accel->x * expected_accel->z;
	error.z = current_accel->x * expected_accel->y - current_accel->y * expected_accel->x;
	float norm = calc_3D_norm(&error);
	if (norm > 1e-6f){
	float inv_norm = 1.0f/norm;
	error.x *= inv_norm;
	error.y *= inv_norm;
	error.z *= inv_norm;
	}
	else {
	error.x = 0.0f;
	error.y = 0.0f;
	error.z = 0.0f;
	}
	return error;
}

static axis_scaled_t angle_speed_correction(const quat_t *q, const axis_scaled_t *gyro, const axis_scaled_t *accel_error, float beta_ratio){
	axis_scaled_t corrected;

	corrected.x = gyro->x + beta_ratio * accel_error->x;
	corrected.y = gyro->y + beta_ratio * accel_error->y;
	corrected.z = gyro->z + beta_ratio * accel_error->z;

	return corrected;
}

static quat_t calc_d_quat(const quat_t *q, const axis_scaled_t *angle_speed, float dt){
	quat_t dq;

	float wx = angle_speed->x;
	float wy = angle_speed->y;
	float wz = angle_speed->z;

	dq.w = 0.5f * (-q->x * wx - q->y * wy - q->z * wz) * dt;
	dq.x = 0.5f * (q->w * wx + q->y * wz - q->z * wy) * dt;
	dq.y = 0.5f * (q->w * wy - q->x * wz + q->z * wx) * dt;
	dq.z = 0.5f * (q->w * wz + q->x * wy - q->y * wx) * dt;

	return dq;
}

static quat_t calc_current_quat(const quat_t *old_q, const quat_t *d_q){
	quat_t new_q;

	new_q.w = old_q->w + d_q->w;
	new_q.x = old_q->x + d_q->x;
	new_q.y = old_q->y + d_q->y;
	new_q.z = old_q->z + d_q->z;
	return normalize_quat(&new_q);
}

static euler_t calc_euler_angles_fixed(const quat_t *q) {
    euler_t angles;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        angles.pitch = copysignf(PI / 2.0f, sinp);
    } else {
        angles.pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return angles;
}

static euler_t calc_euler_angles(const quat_t *q){

	euler_t angles;

	//Roll - крен - вращение по оси X
	float sin_roll = 2.0f * (q->w * q->x + q->y * q->z);
	float cos_roll = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);

	angles.roll = atan2f(sin_roll, cos_roll);

	// Pitch - тангаж - вращение по Y

	float sin_pitch = 2.0f * (q->w * q->y - q->z * q->x);

	if (fabsf(sin_pitch) >= 1.0f){
		angles.pitch = copysignf(PI / 2.0f, sin_pitch);
	}
	else {angles.pitch = asinf(sin_pitch);}

	// Yaw - рыскание - вращение по Z

	float sin_yaw = 2.0f * (q->w * q->z + q->x * q->y);
	float cos_yaw = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);

	angles.yaw = atan2f(sin_yaw, cos_yaw);

	/*angles.yaw /= 4.0f;
	angles.roll /= 4.0f;
	angles.pitch /= 4.0f;*/

	return angles;
}

void madgwick_init(MadgwickFilter *filter, float dt){
	if (!filter) {return;}

	// Начальная ориентация при запуске системы

	filter->orientation.w = 1.0f;
	filter->orientation.x = 0.0f;
	filter->orientation.y = 0.0f;
	filter->orientation.z = 0.0f;

	// Начальное смещение гироскопа

	filter->gyro_bias.x = 0.0f;
	filter->gyro_bias.y = 0.0f;
	filter->gyro_bias.z = 0.0f;

	// Инициализация времени

	filter->last_time = get_time_us();
}

void madgwick_run(MadgwickFilter *filter, const axis_scaled_t *accel, const axis_scaled_t *gyro){
	if (!filter || !accel || !gyro) {return;}

	// Работа с временем (получаем текущее время, вычисляем dt и обновляем время последнего обновления

	//uint32_t current_us = get_time_us();
	//uint32_t dt_us = current_us - filter->last_time;
	//filter->last_time = current_us;

	// Защита от слишком больших dt (первый вызов, инициализация)

	//if (dt_us > 100000){dt_us = 10000;}

	// Защита от слишком маленьких dt

	//if (dt_us < 1000){dt_us = 1000;}

	// конвертация в секунды

	//float dt = dt_us / 1000000.0f;

	// Динамическая компенсация дрейфа гироскопа

	axis_scaled_t gyro_corrected;

	gyro_corrected.x = gyro->x - filter->gyro_bias.x;
	gyro_corrected.y = gyro->y - filter->gyro_bias.y;
	gyro_corrected.z = gyro->z - filter->gyro_bias.z;

	// Вычисление бета коэффициента с учетом dt
	float dt = 0.002f;
	float beta = calc_beta_ratio(accel, dt);

	// Ожидаемое ускорение из текущей ориентации

	axis_scaled_t expected = calc_expected_accel_vector(&filter->orientation);

	// Ошибка между ожидаемым и реальным ускорением

	axis_scaled_t error = calc_accel_vector_error(accel, &expected);

	// Корреция угловой скорости на основе ошибки и бета коэффициента

	axis_scaled_t corrected_angle_speed = angle_speed_correction(&filter->orientation, &gyro_corrected, &error, beta);

	// Медленная адаптация дрейфа гироскопа
	// (Только если акселерометр в нормальных условиях)
	float accel_norm = sqrtf(accel->x * accel->x +
	                         accel->y * accel->y +
	                         accel->z * accel->z);
	 if (accel_norm > 0.8f && accel_norm < 1.2f) {
	 // Очень медленная адаптация (1% от коррекции за секунду)
		 float adapt_rate = 0.02f;
	     filter->gyro_bias.x += error.x * beta * dt * adapt_rate;
	     filter->gyro_bias.y += error.y * beta * dt * adapt_rate;
	     filter->gyro_bias.z += error.z * beta * dt * adapt_rate;

	 // Ограничиваем дрейф разумными пределами
	     const float max_bias = 0.1f;  // 0.1 рад/с = ~5.7°/с
	     if (fabsf(filter->gyro_bias.x) > max_bias)
	            filter->gyro_bias.x = copysignf(max_bias, filter->gyro_bias.x);
	     if (fabsf(filter->gyro_bias.y) > max_bias)
	            filter->gyro_bias.y = copysignf(max_bias, filter->gyro_bias.y);
	     if (fabsf(filter->gyro_bias.z) > max_bias)
	            filter->gyro_bias.z = copysignf(max_bias, filter->gyro_bias.z);
	    }

	 // вычисление малого кватерниона

	 quat_t d_q = calc_d_quat(&filter->orientation, &corrected_angle_speed, dt);

	 // обновление кватерниона

	 filter->orientation = calc_current_quat(&filter->orientation, &d_q);

}

euler_t madgwick_get_euler(const MadgwickFilter *filter){
	if (!filter){
		euler_t zero = {0,0,0};
		return zero;
	}
	return calc_euler_angles(&filter->orientation);
}
