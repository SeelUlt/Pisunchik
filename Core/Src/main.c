/* USER CODE BEGIN Header */
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include "icm20948.h"
#include "madgwick.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
char printf_buffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Функция для перенаправления printf на UART
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// Опционально: для scanf (если нужно чтение)
#ifdef __GNUC__
int __io_getchar(void)
{
    uint8_t ch;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

void print_float(float val){
	int32_t int_part = 0;
	int32_t float_part = 0;
	int_part = (int32_t)val;
	float frac = val - (float)int_part;

	if(frac < 0){frac = -frac;}

	float_part = (int32_t)(frac * 10000.0f + 0.5f);

	if(float_part >= 10000){
		if (val >= 0){int_part++;}
		else {int_part--;}
		float_part = 0;
	}
	printf("%ld.%04ld\n", (long)int_part, (long)float_part);


}

void get_calib_data(){
	axis_scaled_t baby = {};
	uint16_t samples = 300;
	baby = accel_six_dof_calib(samples);
	printf("x axis average value: ");
	print_float(baby.x);
	printf("y axis average value: ");
	print_float(baby.y);
	printf("z axis average value: ");
	print_float(baby.z);

}

void init_fail_detector(init_status status){
	switch(status)
	{
	    case init_success:
	        printf("Init success\r\n");
	        break;

	    case who_am_i_fail:
	        printf("WHO_AM_I failed\r\n");
	        break;

	    case reset_fail:
	        printf("Reset failed\r\n");
	        break;

	    case wake_up_fail:
	        printf("Wake up failed\r\n");
	        break;

	    case clock_source_fail:
	        printf("Clock source config failed\r\n");
	        break;

	    case odr_fail:
	        printf("ODR align failed\r\n");
	        break;

	    case spi_slave_enable_fail:
	        printf("SPI slave enable failed\r\n");
	        break;

	    case gyro_srd_fail:
	        printf("Gyro sample rate divider failed\r\n");
	        break;

	    case gyro_fsf_fail:
	        printf("Gyro full scale config failed\r\n");
	        break;

	    case accel_fsf_fail:
	        printf("Accel full scale config failed\r\n");
	        break;

	    default:
	        printf("Unknown init error\r\n");
	        break;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(spi_cs_port, spi_cs_pin, SET);
  if (who_am_i() == HAL_OK){printf("Im here!\r\n");}
  else {printf("Its so sad\r\n");}
  init_status init_st = icm20948_init(_500dps, _4g, 1);
  if (init_st != init_success){
	  printf("Init error\r\n");
  }
  else{
	  printf("Init success\r\n");
  }
  init_fail_detector(init_st);
  axis_raw_t accel = {};
  axis_raw_t gyro = {};
  axis_scaled_t scaled_accel = {};
  axis_scaled_t scaled_gyro = {};
  MadgwickFilter Filter = {};
  madgwick_init(&Filter, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf(".\r\n");
	  HAL_Delay(2);
	  icm20948_gyro_read(&gyro);
	  icm20948_accel_read(&accel);

	  madgwick_run(&Filter, &scaled_accel, &scaled_gyro);
	  /*printf("G[%6d %6d %6d]  A[%6d %6d %6d]\r\n",
	         gyro.x, gyro.y, gyro.z,
	         accel.x, accel.y, accel.z);*/

    /* USER CODE BEGIN 3 */
  }
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

/* USER CODE BEGIN 4 */

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
