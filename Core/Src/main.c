/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "bmi088.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TTL_HNDLR		huart3
#define SD_CARD_HNDLR	hspi3
#define MAG_I2C_HNDLR	hi2c3
#define IMU_I2C_HNDLR	hi2c1
#define VLT_ADC_HNDLR	hadc1
#define AMP_ADC_HNDLR	hadc2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bmi088_struct_t bmi_imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t bmi_imu_init(void);
static void reg_3v3_on(void);
static void reg_3v3_off(void);
void serial_println(char* str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t str[100];
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  //HAL_Delay(4000);

  reg_3v3_on();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  //imu start
  if(!bmi_imu_init()){
	  serial_println("bmi fail");
  }
  else{
	  serial_println("bmi success");
  }

  bmi088_config(&bmi_imu);

  HAL_GPIO_WritePin(MD_IN_A_GPIO_Port, MD_IN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MD_IN_B_GPIO_Port, MD_IN_B_Pin, GPIO_PIN_RESET);

  uint32_t adc1;
  uint32_t adc2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
/*
	  HAL_ADC_Start(&VLT_ADC_HNDLR);
	  HAL_ADC_Start(&AMP_ADC_HNDLR);

	  HAL_ADC_PollForConversion(&VLT_ADC_HNDLR, 100);
	  adc1 = HAL_ADC_GetValue(&VLT_ADC_HNDLR);
	  float adc1_f = (float)adc1 - 25.0;
	  adc1_f = (adc1_f > 0) * adc1_f;
	  adc1_f	= adc1_f * 5.31;

	  HAL_ADC_PollForConversion(&AMP_ADC_HNDLR, 100);
	  adc2 = HAL_ADC_GetValue(&AMP_ADC_HNDLR);
	  float adc2_f = (float)adc2 * 0.015;

	  sprintf((char*)str, "adc1= %lu\n\r", adc1);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);

	  sprintf((char*)str, "adc2= %.2f\n\r", adc2_f);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);

	  uint8_t chip_id = bmi088_getGyroChipId();
	  sprintf((char*)str, "chip id =  %x\n\r", chip_id);
	  HAL_UART_Transmit(&huart3, str, strlen((char*)str), 50);
*/

//	  sprintf((char*)str, "encoder =  %lu\n\r", (TIM2->CNT)>>2);
//	  HAL_UART_Transmit(&TTL_HNDLR, str, strlen((char*)str), 50);

	  sprintf((char*)str, "bmi accel x =  %f\n\r", bmi_imu.acc_x);
	  HAL_UART_Transmit(&TTL_HNDLR, str, strlen((char*)str), 50);

	  HAL_UART_Transmit(&TTL_HNDLR, (uint8_t*)"---------------------------\r\n", strlen("---------------------------\r\n"), 50);

	  bmi088_update(&bmi_imu);

	  HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	  HAL_Delay(300);

	  HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//BMI sensor initialize function.
uint8_t bmi_imu_init(void)
{
	//Acc config
	bmi_imu.deviceConfig.acc_bandwith = ACC_BWP_OSR4;
	bmi_imu.deviceConfig.acc_outputDateRate = ACC_ODR_200;
	bmi_imu.deviceConfig.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	bmi_imu.deviceConfig.acc_range = ACC_RANGE_12G;

	// Gyro config
	bmi_imu.deviceConfig.gyro_bandWidth = GYRO_BW_230;
	bmi_imu.deviceConfig.gyro_range = GYRO_RANGE_2000;
	bmi_imu.deviceConfig.gyro_powerMode = GYRO_LPM_NORMAL;

	bmi_imu.deviceConfig.acc_IRQ = EXTI9_5_IRQn;
	bmi_imu.deviceConfig.gyro_IRQ = EXTI9_5_IRQn;
	bmi_imu.deviceConfig.BMI_I2c = &IMU_I2C_HNDLR;

	return	bmi088_init(&bmi_imu);
}

//Pcb 3.3 v regulator on function.
void reg_3v3_on()
{
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}

//Pcb 3.3 v regulator off function.
void reg_3v3_off()
{
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
}

void serial_println(char* str)
{

	HAL_UART_Transmit(&TTL_HNDLR, (uint8_t*)str, strlen(str), 50);
	HAL_UART_Transmit(&TTL_HNDLR, (uint8_t*)"\r\n", 2, 50);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT_ACC_Pin)
	{
		bmi_imu.rawDatas.isAccelUpdated = 1;
	}
	if(GPIO_Pin == INT_GYRO_Pin)
	{
		bmi_imu.rawDatas.isGyroUpdated = 1;
	}
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

#ifdef  USE_FULL_ASSERT
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
