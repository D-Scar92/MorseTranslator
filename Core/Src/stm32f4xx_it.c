/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
extern uint8_t morse_word[];
extern uint8_t error_flag;
extern uint8_t signal_counter;
extern uint8_t display_flag;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
/* Need to check which timer generated the interrupt! */
	if(htim->Instance == TIM6){
		if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//A
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 99){		//B
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 0 && morse_word[4] == 99){		//C
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 99 && morse_word[4] == 99){		//D
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 99 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//E
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 0 && morse_word[4] == 99){		//F
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 99 && morse_word[4] == 99){		//G
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 99){		//H
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//I
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 99){		//J
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);

		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 99 && morse_word[4] == 99){		//K
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 99){		//L
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//M
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//N
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 99 && morse_word[4] == 99){		//O
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 0 && morse_word[4] == 99){		//P
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 1 && morse_word[4] == 99){		//Q
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 99 && morse_word[4] == 99){		//R
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 99 && morse_word[4] == 99){		//S
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 99 && morse_word[2] == 99 && morse_word[3] == 99 && morse_word[4] == 99){		//T
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 99 && morse_word[4] == 99){		//U
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 1 && morse_word[4] == 99){		//V
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 99 && morse_word[4] == 99){		//W
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 1 && morse_word[4] == 99){		//X
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 99){		//Y
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 99){		//Z
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 1){		//0
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 1){		//1
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 1){		//2
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 1 && morse_word[4] == 1){		//3
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 1){		//4
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 0){		//5
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 0 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 0){		//6
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 0 && morse_word[3] == 0 && morse_word[4] == 0){		//7
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_RESET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 0 && morse_word[4] == 0){		//8
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 1 && morse_word[1] == 1 && morse_word[2] == 1 && morse_word[3] == 1 && morse_word[4] == 0){		//9
			display_flag = 1;
			HAL_GPIO_WritePin(A_SEG_GPIO_Port, A_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(B_SEG_GPIO_Port, B_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(C_SEG_GPIO_Port, C_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_SEG_GPIO_Port, D_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E_SEG_GPIO_Port, E_SEG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(F_SEG_GPIO_Port, F_SEG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_SEG_GPIO_Port, G_SEG_Pin, GPIO_PIN_SET);
		}
		else if(morse_word[0] == 0 || morse_word[0] == 1)
			error_flag = 1;

		morse_word[0] = 99; morse_word[1] = 99; morse_word[2] = 99; morse_word[3] = 99; morse_word[4] = 99;
		signal_counter = 0;
	}
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
