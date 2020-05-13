/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define ERROR_LED_Pin GPIO_PIN_3
#define ERROR_LED_GPIO_Port GPIOA
#define USER_INPUT_Pin GPIO_PIN_4
#define USER_INPUT_GPIO_Port GPIOA
#define B_SEG_Pin GPIO_PIN_5
#define B_SEG_GPIO_Port GPIOA
#define C_SEG_Pin GPIO_PIN_6
#define C_SEG_GPIO_Port GPIOA
#define D_SEG_Pin GPIO_PIN_7
#define D_SEG_GPIO_Port GPIOA
#define ERROR_Pin GPIO_PIN_0
#define ERROR_GPIO_Port GPIOB
#define ANODE_2_Pin GPIO_PIN_10
#define ANODE_2_GPIO_Port GPIOB
#define F_SEG_Pin GPIO_PIN_7
#define F_SEG_GPIO_Port GPIOC
#define ANODE_1_Pin GPIO_PIN_8
#define ANODE_1_GPIO_Port GPIOA
#define G_SEG_Pin GPIO_PIN_9
#define G_SEG_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ANODE_3_Pin GPIO_PIN_4
#define ANODE_3_GPIO_Port GPIOB
#define ANODE_4_Pin GPIO_PIN_5
#define ANODE_4_GPIO_Port GPIOB
#define E_SEG_Pin GPIO_PIN_6
#define E_SEG_GPIO_Port GPIOB
#define A_SEG_Pin GPIO_PIN_9
#define A_SEG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
