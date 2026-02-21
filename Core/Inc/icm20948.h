#ifndef __ICM20948_H__
#define	__ICM20948_H__
#include "spi.h"
#include <stdbool.h>

#define spi (&hspi1)
#define spi_cs_pin GPIO_PIN_4
#define spi_cs_port GPIOA

#define READ 0x80
#define WRITE 0x00

typedef enum{
	bank_0 = 0 << 4,
	bank_1 = 1 << 4,
	bank_2 = 2 << 4,
	bank_3 = 3 << 4,
} bank;

typedef enum {
	_250dps,
	_500dps,
	_1000dps,
	_2000dps,
} gyro_full_scale;

typedef enum {
	success,
	transmit_fail,
	receive_fail,
	value_not_wrote,
	bank_select_fail
} operation_status;

typedef enum{
	who_am_i_fail,
	reset_fail,
	wake_up_fail,
	clock_source_fail,
	odr_fail,
	spi_slave_enable_fail,
	gyro_srd_fail,
	gyro_fsf_fail,
	accel_fsf_fail,
	init_success
} init_status;


typedef enum {
	yes,
	no
} flag;

typedef enum {
	_2g,
	_4g,
	_8g,
	_16g,
} accel_full_scale;

typedef struct {
	float x;
	float y;
	float z;
} axis_scaled_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} axis_raw_t;

init_status icm20948_init(gyro_full_scale gyro_scale, accel_full_scale accel_scale, uint8_t source);
axis_scaled_t accel_six_dof_calib(uint16_t samples);

void icm20948_accel_read(axis_raw_t* axis);
void icm20948_gyro_read(axis_raw_t* axis);

void icm20948_scale_accel(axis_raw_t *accelRaw, axis_scaled_t *accelScaled);
void icm20948_scale_gyro(axis_raw_t *gyroRaw, axis_scaled_t *gyroScaled);
//void icm20948_gyro_scale_to_dps(axis_raw_t* raw, axis_scaled_t* scaled);
//void icm20948_accel_scale_to_g(axis_raw_t* raw, axis_scaled_t* scaled);

HAL_StatusTypeDef who_am_i();
/*void device_reset();
void sleep();
void wake_up();
void spi_slave_enable();

void clock_source(uint8_t source);
void odr_align_enable();

void gyro_low_pass_filter(uint8_t config); // 0 - 7
void gyro_low_pass_filter(uint8_t config); // 0 - 7

void gyro_sample_rate_divider(uint8_t divider);
void accel_sample_rate_divider(uint8_t divider);

void gyro_full_scale_select(gyro_full_scale full_scale);
void accel_full_scale_select(accel_full_scale full_scale);
*/
void accel_calib();
void gyro_calib();

/* ICM-20948 Registers */
#define ICM20948_ID						0xEA
#define REG_BANK_SEL					0x7F

// USER BANK 0
#define B0_WHO_AM_I						0x00
#define B0_USER_CTRL					0x03
#define B0_LP_CONFIG					0x05
#define B0_PWR_MGMT_1					0x06
#define B0_PWR_MGMT_2					0x07
#define B0_INT_PIN_CFG					0x0F
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17
#define B0_INT_STATUS					0x19
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D
#define B0_ACCEL_XOUT_L					0x2E
#define B0_ACCEL_YOUT_H					0x2F
#define B0_ACCEL_YOUT_L					0x30
#define B0_ACCEL_ZOUT_H					0x31
#define B0_ACCEL_ZOUT_L					0x32
#define B0_GYRO_XOUT_H					0x33
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39
#define B0_TEMP_OUT_L					0x3A
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00
#define B2_GYRO_CONFIG_1				0x01
#define B2_GYRO_CONFIG_2				0x02
#define B2_XG_OFFS_USRH					0x03
#define B2_XG_OFFS_USRL 				0x04
#define B2_YG_OFFS_USRH					0x05
#define B2_YG_OFFS_USRL					0x06
#define B2_ZG_OFFS_USRH					0x07
#define B2_ZG_OFFS_USRL					0x08
#define B2_ODR_ALIGN_EN					0x09
#define B2_ACCEL_SMPLRT_DIV_1			0x10
#define B2_ACCEL_SMPLRT_DIV_2			0x11
#define B2_ACCEL_INTEL_CTRL				0x12
#define B2_ACCEL_WOM_THR				0x13
#define B2_ACCEL_CONFIG					0x14
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

#endif	/* __ICM20948_H__ */
