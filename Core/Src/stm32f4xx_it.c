/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <string.h> // 添加：用于memcpy
#include "control.h"
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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
// 双缓冲区解决方案
extern uint8_t recieve_buf[5];
uint8_t process_buf[5]; // 处理缓冲区
volatile uint8_t data_ready = 0;   // 数据就绪标志

// 蓝牙测试状态量
uint8_t bt_state = 0;

extern int cnt;
int m1 , m2;
int cnt1, cnt2;
uint8_t state1, state2;
uint8_t turn1, turn2;

// 步进电机状态
extern int steppery_step;  // 步进电机1步数_外部
extern int stepperx_step;  // 步进电机2步数_外部
extern int steppery_speed; // 步进电机1速度
extern int stepperx_speed; // 步进电机2速度
extern int steppery_st;   // 步进电机1初始步数
extern int stepperx_st;   // 步进电机2初始步数

extern uint8_t test_if1,test_if2;

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
  while (1)
  {
  }
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
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  // 调参
  HAL_UART_Receive_IT(&huart1, recieve_buf, 5);
  switch (recieve_buf[0])
  {
  case 0x01:
    steppery_step = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x02:
    stepperx_step = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x03:
    steppery_speed = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x04:
    stepperx_speed = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  // case 0x05:
  //   target_V = (int)bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
  //   break;
  // case 0x06:
  //   target_turn = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
  //   break;
  case 0x07:
    m1 = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x08:
    m2 = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x09:
    turn1 = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  case 0x10:
    turn2 = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
    break;
  // case 0x11:
  //   Velocity_Kd = bytes_to_float(recieve_buf[1], recieve_buf[2], recieve_buf[3], recieve_buf[4]);
  //   break;
  default:
    break;
  }

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  // if (cnt <= ABS(stepperx_speed))
  // {
  //   HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5); // 控制步进电机步进

  //   if (stepperx_step > 0 || stepperx_speed > 0)
  //   {
  //     stepper2_st++; // 步进电机2步数增加
  //   }
  //   else if (stepperx_step < 0 || stepperx_speed < 0)
  //   {
  //     stepper2_st--; // 步进电机2步数减少
  //   }
  // }
  
  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  // if (recieve_buf[0] == 0xA5 && recieve_buf[7] == 0x5A)
  // {
  //   if (car_state == 0)
  //   {
  //     car_state = 1;
  //     target_num = recieve_buf[1];
  //   } // 目标病房数字
  //   else if (car_state == 3)
  //   {
  //     if (target_num == 1 || target_num == 2)
  //     {
  //       car_state = 4;
  //       temp_state = 0;
  //     } // 当判断为12病房时直接进入病房
  //     else
  //     {
  //       temp_state = 0;
  //     } // else继续直行
  //   } // 判断距离合适在交叉口处
  //   else if (car_state == 4)
  //   {
  //     if (recieve_buf[1] == 0x00)
  //     {
  //       car_state = 5;
  //       temp_state = 0;
  //     } // 转向角度合适
  //   } // 读取两个数字成功（或者需要左右转）成功则{00，数字左1,数字右1}
  //   uint8_t right_buf[3] = {0xA5, 0x11, 0x5A};
  //   HAL_UART_Transmit(&huart4, right_buf, 3, 1000);
  // }
  // else
  // {
  //   uint8_t err_buf[3] = {0xA5, 0x00, 0x5A};
  //   HAL_UART_Transmit(&huart4, err_buf, 3, 1000);
  //   temp_state = 0;
  // }
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  test_if2++;

  if (stepperx_speed != 0)
  {
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);

    // 步进电机2计数
    if (stepperx_step > 0 || stepperx_speed > 0)
    {
      stepperx_st++; // 步进电机2初始步数增加
    }
    else if (stepperx_step < 0 || stepperx_speed < 0)
    {
      stepperx_st--; // 步进电机2初始步数减少
    }
  }
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  test_if1++;
  if (steppery_speed != 0)
  {
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);

    // 步进电机2计数
    if (steppery_step > 0 || steppery_speed > 0)
    {
      steppery_st++; // 步进电机2初始步数增加
    }
    else if (steppery_step < 0 || steppery_speed < 0)
    {
      steppery_st--; // 步进电机2初始步数减少
    }
  }

  // if (cnt <= ABS(steppery_speed))
  // {
  //   HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2); // 控制步进电机步进

  // 步进电机步数限位
  steppery_speed = (steppery_st >= 1620 || steppery_st <= -10730) ? 0
                                                                    : steppery_speed; // 重置步进电机1速度
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
