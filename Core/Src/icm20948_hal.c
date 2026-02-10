#include "icm20948_hal.h"
#ifndef RPI
#define RPI 1.0f
#endif
// То что static тебя ебать не должно, в main.c используются только не static

static float accel_ratio = 2048.0f;
static float gyro_ratio = 16.4f;

// helper: выбор банка (записываем в регистр 0x7F в bank 0)
static HAL_StatusTypeDef ICM20948_SelectBank(uint8_t bank)
{
    uint8_t tx[2];
    HAL_StatusTypeDef status;

    // регистр BANK_SEL находится в bank 0, адрес 0x7F
    tx[0] = 0x7F & 0x7F; // адрес для записи (MSB=0 -> запись)
    tx[1] = bank & 0x07; // банк 0..3 (лишние биты безопасно отрезаем)

    ICM_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ICM_CS_HIGH();

    //HAL_Delay(1); // небольшая пауза, чтобы банк установился
    return status;
}

HAL_StatusTypeDef WriteReg(uint8_t bank, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;

    // переключаемся в нужный банк, если нужно
    status = ICM20948_SelectBank(bank);
    if (status != HAL_OK) return status;

    uint8_t tx[2];
    tx[0] = reg & 0x7F; // адрес для записи (MSB=0)
    tx[1] = value;

    ICM_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ICM_CS_HIGH();

    //HAL_Delay(1);
    return status;
}

HAL_StatusTypeDef ReadReg(uint8_t bank, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;

    // переключаемся в нужный банк
    status = ICM20948_SelectBank(bank);
    if (status != HAL_OK) return status;

    uint8_t addr = (reg & 0x7F) | 0x80; // MSB=1 -> чтение
    ICM_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, value, 1, HAL_MAX_DELAY);
    }
    ICM_CS_HIGH();

    // вернёмся в bank 0 для предсказуемости
    ICM20948_SelectBank(0);

    return status;
}

static HAL_StatusTypeDef ReadMulti(uint8_t bank, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    status = ICM20948_SelectBank(bank);
    if (status != HAL_OK) return status;

    uint8_t addr = (reg & 0x7F) | 0x80;
    ICM_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    }
    ICM_CS_HIGH();

    ICM20948_SelectBank(0);

    return status;
}


// Пихаем первым аргументов значение из enum accel_range_t,
// вторым аргументом пихаем значение из emun gyro_range_t
HAL_StatusTypeDef ICM20948_Init(accel_range_t accel, gyro_range_t gyro)
{
    uint8_t accel_range = 0x18;  // Будет сдвинуто в биты 4:3
    uint8_t gyro_range = 0x18;   // Будет сдвинуто в биты 4:3
    uint8_t id;

    // Проверяем WHO_AM_I
    if (ReadReg(0, 0x00, &id) != HAL_OK || id != 0xEA) {
        return HAL_ERROR;
    }

    // --- Сброс ---
    WriteReg(0, 0x06, 0x80);  // reset
    HAL_Delay(100);

    // --- Авто такт, отключение sleep ---
    WriteReg(0, 0x06, 0x01);  // auto clock, wake up
    HAL_Delay(100);

    // --- Настройка диапазонов ---
    // выбираем значения диапазона
    switch(accel){
        case ACCEL_2G:  accel_range = 0x00; accel_ratio = 16384.0f; break;  // биты 4:3 = 00
        case ACCEL_4G:  accel_range = 0x08; accel_ratio = 8192.0f;  break;  // биты 4:3 = 01
        case ACCEL_8G:  accel_range = 0x10; accel_ratio = 4096.0f;  break;  // биты 4:3 = 10
        case ACCEL_16G: accel_range = 0x18; accel_ratio = 2048.0f;  break;  // биты 4:3 = 11
        default: accel_range = 0x18; accel_ratio = 2048.0f; break;
    }

    switch(gyro){
        case GYRO_250DPS:  gyro_range = 0x00; gyro_ratio = 131.0f;  break;  // биты 4:3 = 00
        case GYRO_500DPS:  gyro_range = 0x08; gyro_ratio = 65.5f;   break;  // биты 4:3 = 01
        case GYRO_1000DPS: gyro_range = 0x10; gyro_ratio = 32.8f;   break;  // биты 4:3 = 10
        case GYRO_2000DPS: gyro_range = 0x18; gyro_ratio = 16.4f;   break;  // биты 4:3 = 11
        default: gyro_range = 0x18; gyro_ratio = 16.4f; break;
    }

    // --- Отключаем сенсоры перед записью диапазонов ---
    WriteReg(0, 0x07, 0x3F); // PWR_MGMT_2: отключаем все аксель и гироскоп
    HAL_Delay(50);

    // --- Запись диапазонов ---
    uint8_t tmp;

    // Гироскоп
    ReadReg(2, 0x01, &tmp);
    tmp = (tmp & ~0x18) | gyro_range;  // Маска 0x18 = биты 4:3
    WriteReg(2, 0x01, tmp);

    // Акселерометр
    ReadReg(2, 0x14, &tmp);
    tmp = (tmp & ~0x18) | accel_range;  // Маска 0x18 = биты 4:3
    WriteReg(2, 0x14, tmp);

    HAL_Delay(50);

    // --- Включаем сенсоры ---
    WriteReg(0, 0x07, 0x00); // PWR_MGMT_2: включаем все аксель + гироскоп
    HAL_Delay(50);

    return HAL_OK;
}


// в функции типа Read пихаем аргументом структуру типа AxisDataRaw_t
// и она заполняется данными
HAL_StatusTypeDef ICM20948_ReadAccel(AxisDataRaw_t *accel)
{
    uint8_t buf[6];
    if (ReadMulti(0, 0x2D, buf, 6) != HAL_OK) return HAL_ERROR;

    accel->x = (int16_t)(buf[0] << 8 | buf[1]);
    accel->y = (int16_t)(buf[2] << 8 | buf[3]);
    accel->z = (int16_t)(buf[4] << 8 | buf[5]);

    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadGyro(AxisDataRaw_t *gyro)
{
    uint8_t buf[6];
    if (ReadMulti(0, 0x33, buf, 6) != HAL_OK) return HAL_ERROR;

    gyro->x = (int16_t)(buf[0] << 8 | buf[1]);
    gyro->y = (int16_t)(buf[2] << 8 | buf[3]);
    gyro->z = (int16_t)(buf[4] << 8 | buf[5]);

    return HAL_OK;
}
// в функции типа Scale пихаем первым аргументом структуру типа AxisDataRaw_t
// и исходя из нее заполняется структура типа AxisDataScaled_t, которую пихаем вторым аргументом
HAL_StatusTypeDef ICM20948_ScaleAccel(AxisDataRaw_t *accelRaw, AxisDataScaled_t *accelScaled){
	accelScaled->x = (float)(accelRaw->x) / accel_ratio;
	accelScaled->y = (float)(accelRaw->y) / accel_ratio;
	accelScaled->z = (float)(accelRaw->z) / accel_ratio;
	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ScaleGyro(AxisDataRaw_t *gyroRaw, AxisDataScaled_t *gyroScaled)
{
	gyroScaled->x = (float)(gyroRaw->x) / gyro_ratio * RPI;
	gyroScaled->y = (float)(gyroRaw->y) / gyro_ratio * RPI;
	gyroScaled->z = (float)(gyroRaw->z) / gyro_ratio * RPI;
	return HAL_OK;
}

// ===== КАЛИБРОВКА =====
AxisDataScaled_t Axis_ApplyCalibration(AxisDataScaled_t in, const AxisCalib_t *cal)
{
    AxisDataScaled_t out = in;

    if (cal == NULL || cal->valid == 0) {
        return out;
    }

    out.x = (in.x - cal->offset.x) * cal->scale.x;
    out.y = (in.y - cal->offset.y) * cal->scale.y;
    out.z = (in.z - cal->offset.z) * cal->scale.z;

    return out;
}

HAL_StatusTypeDef ICM20948_CalibrateIMU(AxisCalib_t *accelCal,
                                       AxisCalib_t *gyroCal,
                                       uint16_t samples,
                                       uint16_t delay_ms)
{
    if (!accelCal || !gyroCal || samples == 0) {
        return HAL_ERROR;
    }

    // обнуляем и помечаем invalid пока не посчитали
    accelCal->valid = 0;
    gyroCal->valid  = 0;

    // scale по умолчанию = 1
    accelCal->scale.x = 1.0f; accelCal->scale.y = 1.0f; accelCal->scale.z = 1.0f;
    gyroCal->scale.x  = 1.0f; gyroCal->scale.y  = 1.0f; gyroCal->scale.z  = 1.0f;

    // накопители
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    AxisDataRaw_t a_raw, g_raw;
    AxisDataScaled_t a_s, g_s;

    // маленькая пауза, чтобы после init всё стабилизировалось
    HAL_Delay(50);

    for (uint16_t i = 0; i < samples; i++) {
        if (ICM20948_ReadAccel(&a_raw) != HAL_OK) return HAL_ERROR;
        if (ICM20948_ReadGyro(&g_raw) != HAL_OK) return HAL_ERROR;

        if (ICM20948_ScaleAccel(&a_raw, &a_s) != HAL_OK) return HAL_ERROR; // a_s в g
        if (ICM20948_ScaleGyro(&g_raw, &g_s) != HAL_OK) return HAL_ERROR;  // g_s в deg/s

        ax += a_s.x; ay += a_s.y; az += a_s.z;
        gx += g_s.x; gy += g_s.y; gz += g_s.z;

        if (delay_ms) HAL_Delay(delay_ms);
    }

    const float invN = 1.0f / (float)samples;
    const float mean_ax = ax * invN;
    const float mean_ay = ay * invN;
    const float mean_az = az * invN;

    const float mean_gx = gx * invN;
    const float mean_gy = gy * invN;
    const float mean_gz = gz * invN;

    // ---- GYRO: цель 0 ----
    gyroCal->offset.x = mean_gx;
    gyroCal->offset.y = mean_gy;
    gyroCal->offset.z = mean_gz;

    // ---- ACCEL: X/Y -> 0, Z -> ±1g ----
    accelCal->offset.x = mean_ax;
    accelCal->offset.y = mean_ay;

    // хотим, чтобы после калибровки Z был близок к +1g или -1g (как лежит датчик)
    const float target_z = (mean_az >= 0.0f) ? 1.0f : -1.0f;
    accelCal->offset.z = mean_az - target_z;

    accelCal->valid = 1;
    gyroCal->valid  = 1;

    return HAL_OK;
}

