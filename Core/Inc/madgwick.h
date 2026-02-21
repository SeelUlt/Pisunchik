#ifndef MADGWICK_H
#define MADGWICK_H

#include "icm20948.h"
#include <math.h>

// структура для кватерниона
typedef struct {
	float w, x, y, z;
} quat_t;

// структура для углов Эйлера
typedef struct{
	float roll, pitch, yaw;
} euler_t;

typedef struct {
	quat_t orientation;
	axis_scaled_t gyro_bias;
	uint32_t last_time;
} MadgwickFilter;
/*	ЭТОТ РЯД ФУНКЦИЙ БЫЛ ИЗМЕНЕН НА STATIC В СВЯЗИ С ТЕМ ЧТО ДЛЯ ИСПОЛЬЗОВАНИЯ В ДРУГИХ ФАЙЛАХ ТЕПЕРЬ ДОСТУПНЫ ТОЛЬКО
float calc_beta_ratio( //                         MADGWICK_INIT, MADGWICK_RUN, MADGWICK_GET_EULER И MADGWICK_GET_QUAT
	const axis_scaled_t *accels,
	float dt		// принимает структуру с текущими линейными ускорениями и dt и возвращает бета-коэффициент
);

axis_scaled_t calc_expected_accel_vector(
	const quat_t *q // принимает структуру с кватернионом и возвращает структуру
								// с ожидаемыми линейными ускорениями
);

axis_scaled_t calc_accel_vector_error(
	const axis_scaled_t *current_accel, // принимает структуры реального и ожидаемого линейного ускорения
	const axis_scaled_t *expect_accel   // и возвращает структуру с вектором ошибки
);

axis_scaled_t angle_speed_correction(
	const quat_t *q,
	const axis_scaled_t *gyro, // принимает текущие угловые ускорения, вектор ошибки и бета-коэффициент
	const axis_scaled_t *accel_error, // возвращает скоректированные угловые ускорения
	float beta_ratio
);

quat_t calc_d_quat(
	const quat_t *q,
	const axis_scaled_t *angle_speed, // принимает скорректированные угловые ускорения и время между
	float dt							 // измерениями и возвращает вычисленный малый кватернион
);

quat_t calc_current_quat(
	const quat_t *old_q, // принимает старый и малый кватеренионы, возвращает актуальный
	const quat_t *d_q	 // умножение кватернионов
);
quat_t normalize_quat(
	const quat_t *current_q // принимает кватернион и нормализует его
);
euler_t calc_euler_angles(
	const quat_t *q // принимает кватернион и переводит его в углы Эйлера
);
*/
uint32_t get_time_us(void);

void madgwick_init(MadgwickFilter *filter, float dt);
void madgwick_run(MadgwickFilter *filter, const axis_scaled_t *accel, const axis_scaled_t *gyro);
euler_t madgwick_get_euler(const MadgwickFilter *filter);
quat_t madgwick_get_quat(const MadgwickFilter *filter);

#endif
