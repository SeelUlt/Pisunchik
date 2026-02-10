#include "icm20948.h"

static float gyro_scale_factor;
static float accel_scale_factor;
static bank last_bank = bank_3;

static void cs_high();
static void cs_low();

static operation_status select_bank(bank b);

static operation_status read_once_reg(bank b, uint8_t reg, uint8_t* value);
static operation_status write_once_reg(bank b, uint8_t reg, uint8_t value, flag do_check);

static operation_status read_multi_reg(bank b, uint8_t reg, uint8_t len, uint8_t* value);

init_status icm20948_init(gyro_full_scale gyro_scale, accel_full_scale accel_scale){

	// who am i
	if (who_am_i() != HAL_OK){return who_am_i_fail;}
	HAL_Delay(100);

	// imu reset
	if (write_once_reg(bank_0, B0_PWR_MGMT_1, (0x80 | 0x41), yes) != success){return reset_fail;}
	HAL_Delay(100);

	// wake up
	uint8_t check_pwr;
	operation_status wake_read = read_once_reg(bank_0, B0_PWR_MGMT_1, &check_pwr);
	check_pwr &= 0xBF;
	operation_status wake_write = write_once_reg(bank_0, B0_PWR_MGMT_1, check_pwr, yes);
	if ((wake_read != success)||(wake_write != success)){return wake_up_fail;}
	HAL_Delay(100);


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
	uint8_t read_reg = READ | reg;
	if(select_bank(b) != success){return bank_select_fail;}
	cs_low();
	if (HAL_SPI_Transmit(spi, &read_reg, 1, 1000) != HAL_OK){
		cs_high();
		return transmit_fail;}
	if (HAL_SPI_Receive(spi, value, len, 1000) != HAL_OK){
		cs_high();
		return receive_fail;}
	cs_high();
	return success;
}



