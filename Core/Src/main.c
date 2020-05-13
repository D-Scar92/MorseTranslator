/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
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
uint8_t morse_word [5]= {99,99,99,99,99};
uint8_t signal_counter = 0;
uint8_t error_flag = 0;
uint8_t display_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void anode_handler();
void error_handler();
void input_handler();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void anode_handler(){
	static uint8_t display_counter = 1;
	static uint8_t init_var = 1;
	if(init_var){
		HAL_GPIO_WritePin(ANODE_1_GPIO_Port, ANODE_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_2_GPIO_Port, ANODE_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_3_GPIO_Port, ANODE_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_4_GPIO_Port, ANODE_4_Pin, GPIO_PIN_SET);
		init_var = 0;
	}
	if(display_counter == 1){
		HAL_GPIO_WritePin(ANODE_1_GPIO_Port, ANODE_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ANODE_4_GPIO_Port, ANODE_4_Pin, GPIO_PIN_SET);
	}
	if(display_counter == 2){
		HAL_GPIO_WritePin(ANODE_1_GPIO_Port, ANODE_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_2_GPIO_Port, ANODE_2_Pin, GPIO_PIN_RESET);
	}
	if(display_counter == 3){
		HAL_GPIO_WritePin(ANODE_2_GPIO_Port, ANODE_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_3_GPIO_Port, ANODE_3_Pin, GPIO_PIN_RESET);
	}
	if(display_counter == 4){
		HAL_GPIO_WritePin(ANODE_3_GPIO_Port, ANODE_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ANODE_4_GPIO_Port, ANODE_4_Pin, GPIO_PIN_RESET);
	}
	display_counter++;
	if(display_counter > 4)
		display_counter = 1;
	display_flag = 0;
	return;
}

void error_handler(){
	HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	error_flag = 0;
}

void input_handler(){
	HAL_TIM_Base_Stop_IT(&htim6);
	uint32_t t1 = HAL_GetTick();
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	while(HAL_GPIO_ReadPin(USER_INPUT_GPIO_Port, USER_INPUT_Pin)){
		  continue;
	}
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	uint32_t t2 = HAL_GetTick();
	uint32_t dt = t2 - t1;
	if(dt > 300){
		  morse_word[signal_counter] = 1;
	}
	else{
		  morse_word[signal_counter] = 0;
	}
	signal_counter++;
	 __HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start_IT(&htim6);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(error_flag){
		  error_handler();
	  }
	  if(display_flag){
		  anode_handler();
	  }
	  if(HAL_GPIO_ReadPin(USER_INPUT_GPIO_Port, USER_INPUT_Pin)){
		  input_handler();
	  }

  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
