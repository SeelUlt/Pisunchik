#include "icm20948.h"
#define PI_180 0.0174532925f
static float gyro_scale_factor;
static float accel_scale_factor;
static bank last_bank = bank_3;

static void cs_high();
static void cs_low();

static operation_status select_bank(bank b);

static operation_status read_once_reg(bank b, uint8_t reg, uint8_t* value);
static operation_status write_once_reg(bank b, uint8_t reg, uint8_t value, flag do_check);

static operation_status read_multi_reg(bank b, uint8_t reg, uint8_t len, uint8_t* value);

init_status icm20948_init(gyro_full_scale gyro_scale, accel_full_scale accel_scale, uint8_t source){

	// who am i
	if (who_am_i() != HAL_OK){return who_am_i_fail;}
	HAL_Delay(100);

	// imu reset
	if (write_once_reg(bank_0, B0_PWR_MGMT_1, (0x80 | 0x41), no) != success){return reset_fail;}
	HAL_Delay(100);

	// wake up
	uint8_t check_pwr;
	operation_status check_read = read_once_reg(bank_0, B0_PWR_MGMT_1, &check_pwr);
	check_pwr &= 0xBF;
	operation_status check_write = write_once_reg(bank_0, B0_PWR_MGMT_1, check_pwr, no);
	if ((check_read != success)||(check_write != success)){return wake_up_fail;}
	HAL_Delay(100);

	// clock source
	uint8_t check_clock;
	check_read = read_once_reg(bank_0, B0_PWR_MGMT_1, &check_clock);
	check_clock |= source;
	check_write = write_once_reg(bank_0, B0_PWR_MGMT_1, check_clock, no);
	if((check_read != success)||(check_write != success)){return clock_source_fail;}
	HAL_Delay(100);

	// odr align turn on
	check_write = write_once_reg(bank_2, B2_ODR_ALIGN_EN, 0x01, no);
	if (check_write != success){return odr_fail;}

	// spi slave mode turn on
	uint8_t check_spi;
	check_read = read_once_reg(bank_0, B0_USER_CTRL, &check_spi);
	check_spi |= 0x10;
	check_write = write_once_reg(bank_0, B0_USER_CTRL, check_spi, no);
	if((check_read != success)||(check_write != success)){return spi_slave_enable_fail;}

	// sample rate divider in future

	// low pass lilter in future

	// gyro full scale select
	uint8_t gyro_fsf_check;
	check_read = read_once_reg(bank_2, B2_GYRO_CONFIG_1, &gyro_fsf_check);
	switch(gyro_scale)
		{
			case _250dps :
				gyro_fsf_check |= 0x00;
				gyro_scale_factor = 131.0;
				break;
			case _500dps :
				gyro_fsf_check |= 0x02;
				gyro_scale_factor = 65.5;
				break;
			case _1000dps :
				gyro_fsf_check |= 0x04;
				gyro_scale_factor = 32.8;
				break;
			case _2000dps :
				gyro_fsf_check |= 0x06;
				gyro_scale_factor = 16.4;
				break;
		}
	check_write = write_once_reg(bank_2, B2_GYRO_CONFIG_1, gyro_fsf_check, no);
	if((check_read != success)||(check_write != success)){return gyro_fsf_fail;}

	// accel full scale select
	uint8_t accel_fsf_check;
		check_read = read_once_reg(bank_2, B2_ACCEL_CONFIG, &accel_fsf_check);
		switch(accel_scale)
			{
				case _2g :
					accel_fsf_check |= 0x00;
					accel_scale_factor = 16384;
					break;
				case _4g :
					accel_fsf_check |= 0x02;
					accel_scale_factor = 8192;
					break;
				case _8g :
					accel_fsf_check |= 0x04;
					accel_scale_factor = 4096;
					break;
				case _16g :
					accel_fsf_check |= 0x06;
					accel_scale_factor = 2048;
					break;
			}
		check_write = write_once_reg(bank_2, B2_ACCEL_CONFIG, accel_fsf_check, no);
		if((check_read != success)||(check_write != success)){return accel_fsf_fail;}


	// if all chesks passed
	return init_success;
}

HAL_StatusTypeDef who_am_i(){
	uint8_t ok = 0x10;
	read_once_reg(bank_0, 0x00, &ok);
		if (ok == ICM20948_ID){return HAL_OK;}
		else  {return HAL_ERROR;}
}


static void cs_low(){
	HAL_GPIO_WritePin(spi_cs_port, spi_cs_pin, RESET);
}

static void cs_high(){
	HAL_GPIO_WritePin(spi_cs_port, spi_cs_pin, SET);
}

static operation_status select_bank(bank b){
	if(last_bank==b){
	return success;
	}
	else
	{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = b;
	HAL_StatusTypeDef status;
	cs_low();
	status = HAL_SPI_Transmit(spi, write_reg, 2, 10);
	cs_high();
	if (status == HAL_OK){
		last_bank = b;
		return success;
		}
	else {
		return bank_select_fail;
		}
	}
}

static operation_status write_once_reg(bank b, uint8_t reg, uint8_t value, flag do_check){
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = value;
	if(select_bank(b) != success){return bank_select_fail;}
	cs_low();
	if (HAL_SPI_Transmit(spi, write_reg, 2, 1000) != HAL_OK){
		cs_high();
		return transmit_fail;}
	cs_high();
	if (do_check == yes){
		uint8_t check_buf;
		uint8_t read_reg = READ | reg;
		cs_low();
		HAL_SPI_Transmit(spi, &read_reg, 1, 1000); ////////// добавить проверки
		cs_high();
		HAL_Delay(100);
		cs_low();
		HAL_SPI_Receive(spi, &check_buf, 1, 1000); ////////// если понадобится
		cs_high();
		if (check_buf != write_reg[1]){return value_not_wrote;}
	}
	return success;
}

static operation_status read_once_reg(bank b, uint8_t reg, uint8_t* value){
	uint8_t read_reg = READ | reg;
	if(select_bank(b) != success){return bank_select_fail;}
	cs_low();
	if (HAL_SPI_Transmit(spi, &read_reg, 1, 1000) != HAL_OK){
		cs_high();
		return transmit_fail;}
	if (HAL_SPI_Receive(spi, value, 1, 1000) != HAL_OK){
		cs_high();
		return receive_fail;}
	cs_high();
	return success;
}

static operation_status read_multi_reg(bank b, uint8_t reg, uint8_t len, uint8_t* value){
    if (len == 0) return success; // ничего не делать

    // Убедимся, что можем перейти в нужный bank
    if (select_bank(b) != success) return bank_select_fail;

    uint8_t cmd = READ | reg;
    uint8_t dummy_rx = 0;

    cs_low();

    // Отправляем команду чтения (одновременно принимаем "пустой" байт)
    if (HAL_SPI_TransmitReceive(spi, &cmd, &dummy_rx, 1, 1000) != HAL_OK) {
        cs_high();
        return transmit_fail;
    }

    // Последовательно считываем len байт, отправляя 0x00 и принимая данные
    for (uint8_t i = 0; i < len; ++i) {
        uint8_t tx = 0x00;
        uint8_t rx = 0x00;
        if (HAL_SPI_TransmitReceive(spi, &tx, &rx, 1, 1000) != HAL_OK) {
            cs_high();
            return receive_fail;
        }
        value[i] = rx;
    }

    cs_high();
    return success;
}

// Структура, которую ты прислал:
// typedef struct { int16_t x; int16_t y; int16_t z; } axis_raw_t;

void icm20948_accel_read(axis_raw_t* axis){
    if (axis == NULL) return;

    uint8_t buf[6];
    operation_status st = read_multi_reg(bank_0, B0_ACCEL_XOUT_H, 6, buf);
    if (st != success){
        // диагностическая информация — можешь убрать/заменить на другой лог
        //printf("accel read failed (%d)\r\n", (int)st);
        axis->x = axis->y = axis->z = 0;
        return;
    }

    axis->x = (int16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
    axis->y = (int16_t)((uint16_t)buf[2] << 8 | (uint16_t)buf[3]);
    axis->z = (int16_t)((uint16_t)buf[4] << 8 | (uint16_t)buf[5]);
}

void icm20948_gyro_read(axis_raw_t* axis){
    if (axis == NULL) return;

    uint8_t buf[6];
    operation_status st = read_multi_reg(bank_0, B0_GYRO_XOUT_H, 6, buf);
    if (st != success){
        //printf("gyro read failed (%d)\r\n", (int)st);
        axis->x = axis->y = axis->z = 0;
        return;
    }

    axis->x = (int16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
    axis->y = (int16_t)((uint16_t)buf[2] << 8 | (uint16_t)buf[3]);
    axis->z = (int16_t)((uint16_t)buf[4] << 8 | (uint16_t)buf[5]);
}

void icm20948_scale_accel(axis_raw_t *accelRaw, axis_scaled_t *accelScaled){
	accelScaled->x = (float)(accelRaw->x) / accel_scale_factor;
	accelScaled->y = (float)(accelRaw->y) / accel_scale_factor;
	accelScaled->z = (float)(accelRaw->z) / accel_scale_factor;
	return;
}

void icm20948_scale_gyro(axis_raw_t *gyroRaw, axis_scaled_t *gyroScaled)
{
	float _gyro_scale_factor = gyro_scale_factor *= PI_180;
	gyroScaled->x = (float)(gyroRaw->x) / _gyro_scale_factor;
	gyroScaled->y = (float)(gyroRaw->y) / _gyro_scale_factor;
	gyroScaled->z = (float)(gyroRaw->z) / _gyro_scale_factor;
	return;
}

axis_scaled_t accel_six_dof_calib(uint16_t samples){
	axis_raw_t axis_raw_buf = {};
	axis_scaled_t axis_scaled_buf = {};
	axis_scaled_t axis_counter = {};
	axis_counter.x=0;
	axis_counter.y=0;
	axis_counter.z=0;
	int16_t counter = 0;
	for (int16_t i = 0; i < samples; i++){
		icm20948_accel_read(&axis_raw_buf);
		icm20948_scale_accel(&axis_raw_buf, &axis_scaled_buf);
		axis_counter.x += axis_scaled_buf.x;
		axis_counter.y += axis_scaled_buf.y;
		axis_counter.z += axis_scaled_buf.z;
		counter+=1;
	}
	axis_counter.x /= counter;
	axis_counter.y /= counter;
	axis_counter.z /= counter;
	return axis_counter;
}


