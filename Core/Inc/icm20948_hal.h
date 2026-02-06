#ifndef ICM20948_HAL_H
#define ICM20948_HAL_H

#include "stm32f1xx_hal.h"

// основная структура, в которой будут храниться данные с трех осей для одного типа ускорений в СЫРОМ виде
typedef struct {
    int16_t x, y, z;
} AxisDataRaw_t;
// структура для хранения данных по трем осям после приведения к СИ
typedef struct {
	float x, y, z;
} AxisDataScaled_t;
// диапазоны измерений для IMU, в случае некорректного указания диапазонов по default будут выбраны максимальные диапазоны
typedef enum {
	ACCEL_2G = 2,
	ACCEL_4G = 4,
	ACCEL_8G = 8,
	ACCEL_16G = 16,
} accel_range_t;

typedef enum {
	GYRO_250DPS = 250,
	GYRO_500DPS = 500,
	GYRO_1000DPS = 1000,
	GYRO_2000DPS = 2000,
} gyro_range_t;

extern SPI_HandleTypeDef hspi1; // берем из main.c hspi1(это структура), чтобы с его помощью использовать spi

// CS на PA4
#define ICM_CS_GPIO_Port GPIOA
#define ICM_CS_Pin       GPIO_PIN_4
// макросы для быстрого использования подтяжки и стяжки CS
#define ICM_CS_LOW()     HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)

// Публичные функции
HAL_StatusTypeDef ICM20948_Init(accel_range_t accel, gyro_range_t gyro); // инициализация датчика, who I am, установка диапазонов измерений
HAL_StatusTypeDef ICM20948_ReadAccel(AxisDataRaw_t *accel); // чтение данных акселерометра
HAL_StatusTypeDef ICM20948_ReadGyro(AxisDataRaw_t *gyro); // чтение данных гироскопа
HAL_StatusTypeDef ICM20948_ScaleGyro(AxisDataRaw_t *gyroRaw, AxisDataScaled_t *gyroScaled); // перевод данных гироскопа в СИ
HAL_StatusTypeDef ICM20948_ScaleAccel(AxisDataRaw_t *accelRaw, AxisDataScaled_t *accelScaled); // перевод данных акселерометра в СИ

// ===== КАЛИБРОВКА =====
typedef struct {
    AxisDataScaled_t offset;   // bias (смещение)
    AxisDataScaled_t scale;    // scale (обычно 1.0)
    uint8_t valid;             // 1 если калибровка рассчитана
} AxisCalib_t;

/**
 * @brief Калибровка IMU в покое.
 *        ВАЖНО: во время калибровки датчик должен лежать неподвижно.
 * @param accelCal  выход: калибровка акселя (в g)
 * @param gyroCal   выход: калибровка гиро (в deg/s, т.к. ScaleGyro возвращает deg/s)
 * @param samples   сколько сэмплов усреднять
 * @param delay_ms  задержка между сэмплами (обычно 2-10 мс)
 */
HAL_StatusTypeDef ICM20948_CalibrateIMU(AxisCalib_t *accelCal, AxisCalib_t *gyroCal, uint16_t samples, uint16_t delay_ms);

/**
 * @brief Применить калибровку к данным по осям.
 *        out = (in - offset) * scale
 */
AxisDataScaled_t Axis_ApplyCalibration(AxisDataScaled_t in, const AxisCalib_t *cal);

#endif
