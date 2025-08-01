/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Stepper_1_en1_Pin|Stepper_1_dir1_Pin|Stepper_1_st1_Pin|Stepper_1_en2_Pin
                          |Stepper_1_dir2_Pin|Stepper_1_st2_Pin|Stepper_2_en1_Pin|Stepper_2_dir1_Pin
                          |Stepper_2_st1_Pin|Stepper_2_en2_Pin|Stepper_2_dir2_Pin|Stepper_2_st2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasound_Trig_GPIO_Port, Ultrasound_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motor_frontB_in1_Pin|motor_frontB_in2_Pin|motor_backA_in2_Pin|motor_backA_in1_Pin
                          |motor_backB_in1_Pin|motor_backB_in2_Pin|motor_frontA_in2_Pin|motor_frontA_in1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper_1_en1_Pin Stepper_1_dir1_Pin Stepper_1_st1_Pin Stepper_1_en2_Pin
                           Stepper_1_dir2_Pin Stepper_1_st2_Pin Stepper_2_en1_Pin Stepper_2_dir1_Pin
                           Stepper_2_st1_Pin Stepper_2_en2_Pin Stepper_2_dir2_Pin Stepper_2_st2_Pin */
  GPIO_InitStruct.Pin = Stepper_1_en1_Pin|Stepper_1_dir1_Pin|Stepper_1_st1_Pin|Stepper_1_en2_Pin
                          |Stepper_1_dir2_Pin|Stepper_1_st2_Pin|Stepper_2_en1_Pin|Stepper_2_dir1_Pin
                          |Stepper_2_st1_Pin|Stepper_2_en2_Pin|Stepper_2_dir2_Pin|Stepper_2_st2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasound_Trig_Pin */
  GPIO_InitStruct.Pin = Ultrasound_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasound_Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_frontB_in1_Pin motor_frontB_in2_Pin motor_backA_in2_Pin motor_backA_in1_Pin
                           motor_backB_in1_Pin motor_backB_in2_Pin motor_frontA_in2_Pin motor_frontA_in1_Pin */
  GPIO_InitStruct.Pin = motor_frontB_in1_Pin|motor_frontB_in2_Pin|motor_backA_in2_Pin|motor_backA_in1_Pin
                          |motor_backB_in1_Pin|motor_backB_in2_Pin|motor_frontA_in2_Pin|motor_frontA_in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : key1_Pin key2_Pin key3_Pin key4_Pin */
  GPIO_InitStruct.Pin = key1_Pin|key2_Pin|key3_Pin|key4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
