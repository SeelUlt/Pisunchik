/* USER CODE BEGIN Header */

// основная инфа по фунциям описана в этом файле и немного подробностей в
// madgwick.c и icm20948.c

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "icm20948_hal.h"
#include "madgwick.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
AxisCalib_t accelCal, gyroCal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// прототип printf для отправки логов по uart
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
#ifndef G
#define G 9.80665f
#endif
#ifndef PI_180
#define PI_180 0.0174532925f
#endif
#ifndef PI_reverse_180
#define PI_reverse_180 57.2957795f
#endif
// прототип printf для отправки логов по uart
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//////////////////////////////
//калибровка imu
/////////////////////////////

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 3 */
	  HAL_Init(); // инициализация миксовских приколюх
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_SPI1_Init();
	  MX_USART1_UART_Init();
	  printf("\r\n\r\n=== ICM-20948 SPI Test on STM32F103 ===\r\n");
	  MadgwickFilter ahrs_filter; // создаем структуру для хранения состояния фильтра
	  AxisDataScaled_t accel_calibrated, gyro_calibrated; // создаем структуры для хранения приведенных в g и рад/с данных imu
	  AxisDataRaw_t accel_raw, gyro_raw, accel, gyro; // создаем структуры для хранения сырых данных imu
	  ICM_CS_HIGH(); // ставим высокий уровень для корректной работы CS
	  madgwick_init(&ahrs_filter, 0.02); // инициализируем фильтр, параметры инициализации хранятся в madgwick.c
	  //uint8_t id = 0; рудимент		 // 0.02 на данный момент является рудиментов, но может пригодиться в будущем

	  if (ICM20948_Init(ACCEL_8G, GYRO_1000DPS) == HAL_OK) { // здесь мы проводим инициализацию imu
	      printf("Initialization successful!\r\n\r\n"); // и проверяем успешная ли она
	  }
	  else {
		   printf("Init failed\r\n");
	  }

	  printf("Starting data output...\r\n");

	  HAL_Delay(200);   // дать IMU стабилизироваться

	  // ===== КАЛИБРОВКА (датчик должен лежать неподвижно) =====
	  printf("Starting IMU calibration...\r\n");
	  if (ICM20948_CalibrateIMU(&accelCal, &gyroCal, 500, 5) == HAL_OK) {
	      printf("IMU calibration OK\r\n");
	      printf("ACC offset (mg): %d %d %d\r\n",
	             (int)(accelCal.offset.x * 1000.0f),
	             (int)(accelCal.offset.y * 1000.0f),
	             (int)(accelCal.offset.z * 1000.0f));

	      printf("GYR offset (mdps): %d %d %d\r\n",
	             (int)(gyroCal.offset.x * 1000.0f),
	             (int)(gyroCal.offset.y * 1000.0f),
	             (int)(gyroCal.offset.z * 1000.0f));

	  } else {
	      printf("IMU calibration FAILED\r\n");
	  }

	  while (1)
	  {
		  ////////////////////////////////// Это добро читает даннные сырые данные imu и выводит их в консоль,
		  ////////////////////////////////// Использую чтобы было видно, работает ли imu, когда не работает фильтр
		  /*if (ICM20948_ReadAccel(&accel) == HAL_OK && ICM20948_ReadGyro(&gyro) == HAL_OK) {
		  	      printf("A: %6d %6d %6d | G: %6d %6d %6d\r\n",
		  	             accel.x, accel.y, accel.z,
		  	             gyro.x, gyro.y, gyro.z);
		  	    } else {
		  	      printf("Read failed\r\n");}
*/
		  //////////////////////////////////
		  /*							  */
		  //////////////////////////////////

		  ////////////////////////////////// Это добро читает сырые данные и после приводит их к g и градусам/с
		  if(ICM20948_ReadAccel(&accel_raw) == HAL_OK && ICM20948_ReadGyro(&gyro_raw) == HAL_OK){
			  if(ICM20948_ScaleAccel(&accel_raw, &accel_calibrated) == HAL_OK && ICM20948_ScaleGyro(&gyro_raw, &gyro_calibrated) == HAL_OK){

				  // применяем калибровку (в тех единицах, что вернул Scale*)
				  accel_calibrated = Axis_ApplyCalibration(accel_calibrated, &accelCal);
				  gyro_calibrated  = Axis_ApplyCalibration(gyro_calibrated,  &gyroCal);

				  //printf("ICM data get success\r\n");
				  gyro_calibrated.x *= PI_180;
				  gyro_calibrated.y *= PI_180;
				  gyro_calibrated.z *= PI_180;

				  madgwick_run(&ahrs_filter, &accel_calibrated, &gyro_calibrated); // функция которая заставляет алгоритм работать
				  euler_t angles = madgwick_get_euler(&ahrs_filter); // получаем из данных фильтра углы Эйлера, то есть текущее положение
				  float roll_deg = angles.roll * PI_reverse_180;	 // датчика в градусах
				  float pitch_deg = angles.pitch * PI_reverse_180;
				  float yaw_deg = angles.yaw * PI_reverse_180;

				  printf("Roll: %6d Pitch: %6d Yaw: %6d\r\n", (int)(roll_deg), (int)(pitch_deg), (int)(yaw_deg));

			}
			  else{printf("ICM scale error\r\n");}
		  }
			else{printf("ICM read error\r\n");}

		  HAL_Delay(2);
	  }

// напоминалка как юзать icm драйвер
	  /*  if (ICM20948_ReadAccel(&accel) == HAL_OK && ICM20948_ReadGyro(&gyro) == HAL_OK) {
	      printf("A: %6d %6d %6d | G: %6d %6d %6d\r\n",
	             accel.x, accel.y, accel.z,
	             gyro.x, gyro.y, gyro.z);
	    } else {
	      printf("Read failed\r\n");
	    }
*/

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /* Настройка PA4 как выход для CS */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* Устанавливаем CS в high по умолчанию */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void fast_check(void)
{
	printf("SystemCoreCLock = %lu Hz\r\n", SystemCoreClock);

	uint32_t sws = RCC->CFGR & RCC_CFGR_SWS;

	if (sws == RCC_CFGR_SWS_PLL) {
	    printf("SYSCLK source: PLL\r\n");
	} else if (sws == RCC_CFGR_SWS_HSI) {
	    printf("SYSCLK source: HSI\r\n");
	} else if (sws == RCC_CFGR_SWS_HSE) {
	    printf("SYSCLK source: HSE\r\n");
	} else {
	    printf("SYSCLK source: UNKNOWN\r\n");
	}

}

void six_dof_calibrate(void){
	AxisDataRaw_t six_raw_accel;
	AxisDataScaled_t six_scaled_accel;
	float x_counter, y_counter, z_counter = 0.0f;
	for (int i = 0; i < 300; i++){
		if (ICM20948_ReadAccel(&six_raw_accel) == HAL_OK){
			if (ICM20948_ScaleAccel(&six_raw_accel, &six_scaled_accel) == HAL_OK){
				x_counter += six_scaled_accel.x;
				y_counter += six_scaled_accel.y;
				z_counter += six_scaled_accel.z;
				HAL_Delay(2);
				if (i % 30 == 0){printf("%d\r\n", i/3);}
			}

		}
	}
	x_counter /= 300.0f;
	y_counter /= 300.0f;
	z_counter /= 300.0f;

	int x_int = (int)x_counter;
	int x_frac = (int)(fabsf(x_counter - x_int) * 10000);

	int y_int = (int)y_counter;
	int y_frac = (int)(fabsf(y_counter - y_int) * 10000);

	int z_int = (int)z_counter;
	int z_frac = (int)(fabsf(z_counter - z_int) * 10000);

	printf("x average = %d.%04d\r\n", x_int, x_frac);
	printf("y average = %d.%04d\r\n", y_int, y_frac);
	printf("z average = %d.%04d\r\n", z_int, z_frac);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


