/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

// 蓝牙接收数组
extern uint8_t recieve_buf[5];
// 蓝牙发送数组
extern uint8_t transmit_buf[20];
//视觉接收数组
extern uint8_t RxCamera_buf[16];
//视觉发送数组
extern uint8_t TxCamera_buf[6];

// 临时字符串
extern uint8_t temp_ZFC[20];

// 视觉接收数组
extern uint8_t carecieve_buf[16],rxprocess_buf[20];


//角加速度
extern float Ax,Ay,Az;
//角速度
extern float Gx,Gy,Gz;
//角度（自动解算）
extern float Roll_x, Pitch_y, Yaw_z;
extern float Kp, Kd, Ki;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
float bytes_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
void cam_receive(void);
void cam_process(void);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BM1_Pin GPIO_PIN_5
#define BM1_GPIO_Port GPIOE
#define BM2_Pin GPIO_PIN_6
#define BM2_GPIO_Port GPIOE
#define Gyroscope_SDA_Pin GPIO_PIN_0
#define Gyroscope_SDA_GPIO_Port GPIOF
#define Gyroscope_SCL_Pin GPIO_PIN_1
#define Gyroscope_SCL_GPIO_Port GPIOF
#define adc_1_Pin GPIO_PIN_0
#define adc_1_GPIO_Port GPIOA
#define adc_2_Pin GPIO_PIN_1
#define adc_2_GPIO_Port GPIOA
#define adc_3_Pin GPIO_PIN_2
#define adc_3_GPIO_Port GPIOA
#define adc_4_Pin GPIO_PIN_3
#define adc_4_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOA
#define motor_frontA_input1_Pin GPIO_PIN_5
#define motor_frontA_input1_GPIO_Port GPIOA
#define motor_frontA_PWM_Pin GPIO_PIN_6
#define motor_frontA_PWM_GPIO_Port GPIOA
#define motor_frontB_PWM_Pin GPIO_PIN_7
#define motor_frontB_PWM_GPIO_Port GPIOA
#define motor_backA_PWM_Pin GPIO_PIN_0
#define motor_backA_PWM_GPIO_Port GPIOB
#define motor_backB_PWM_Pin GPIO_PIN_1
#define motor_backB_PWM_GPIO_Port GPIOB
#define Stepper_1_en1_Pin GPIO_PIN_0
#define Stepper_1_en1_GPIO_Port GPIOG
#define Stepper_1_dir1_Pin GPIO_PIN_1
#define Stepper_1_dir1_GPIO_Port GPIOG
#define Ultrasound_Echo_Pin GPIO_PIN_13
#define Ultrasound_Echo_GPIO_Port GPIOE
#define Ultrasound_Trig_Pin GPIO_PIN_14
#define Ultrasound_Trig_GPIO_Port GPIOE
#define motor_frontB_in1_Pin GPIO_PIN_10
#define motor_frontB_in1_GPIO_Port GPIOB
#define motor_frontB_in2_Pin GPIO_PIN_11
#define motor_frontB_in2_GPIO_Port GPIOB
#define motor_backA_in2_Pin GPIO_PIN_12
#define motor_backA_in2_GPIO_Port GPIOB
#define motor_backA_in1_Pin GPIO_PIN_13
#define motor_backA_in1_GPIO_Port GPIOB
#define motor_backB_in1_Pin GPIO_PIN_14
#define motor_backB_in1_GPIO_Port GPIOB
#define motor_backB_in2_Pin GPIO_PIN_15
#define motor_backB_in2_GPIO_Port GPIOB
#define Gyroscope_TX_Pin GPIO_PIN_8
#define Gyroscope_TX_GPIO_Port GPIOD
#define Gyroscope_RX_Pin GPIO_PIN_9
#define Gyroscope_RX_GPIO_Port GPIOD
#define SG1_Pin GPIO_PIN_12
#define SG1_GPIO_Port GPIOD
#define SG2_Pin GPIO_PIN_13
#define SG2_GPIO_Port GPIOD
#define SG3_Pin GPIO_PIN_14
#define SG3_GPIO_Port GPIOD
#define SG4_Pin GPIO_PIN_15
#define SG4_GPIO_Port GPIOD
#define Stepper_1_st1_Pin GPIO_PIN_2
#define Stepper_1_st1_GPIO_Port GPIOG
#define Stepper_1_en2_Pin GPIO_PIN_3
#define Stepper_1_en2_GPIO_Port GPIOG
#define Stepper_1_dir2_Pin GPIO_PIN_4
#define Stepper_1_dir2_GPIO_Port GPIOG
#define Stepper_1_st2_Pin GPIO_PIN_5
#define Stepper_1_st2_GPIO_Port GPIOG
#define Stepper_2_en1_Pin GPIO_PIN_6
#define Stepper_2_en1_GPIO_Port GPIOG
#define Stepper_2_dir1_Pin GPIO_PIN_7
#define Stepper_2_dir1_GPIO_Port GPIOG
#define Stepper_2_st1_Pin GPIO_PIN_8
#define Stepper_2_st1_GPIO_Port GPIOG
#define Track_SDA_Pin GPIO_PIN_9
#define Track_SDA_GPIO_Port GPIOC
#define Track_SCL_Pin GPIO_PIN_8
#define Track_SCL_GPIO_Port GPIOA
#define Bluetooth_TX_Pin GPIO_PIN_9
#define Bluetooth_TX_GPIO_Port GPIOA
#define Bluetooth_RX_Pin GPIO_PIN_10
#define Bluetooth_RX_GPIO_Port GPIOA
#define Camera_TX_Pin GPIO_PIN_10
#define Camera_TX_GPIO_Port GPIOC
#define Camera_RX_Pin GPIO_PIN_11
#define Camera_RX_GPIO_Port GPIOC
#define Sceen_TX_Pin GPIO_PIN_12
#define Sceen_TX_GPIO_Port GPIOC
#define Sceen_RX_Pin GPIO_PIN_2
#define Sceen_RX_GPIO_Port GPIOD
#define CH340_TX_Pin GPIO_PIN_5
#define CH340_TX_GPIO_Port GPIOD
#define CH340_RX_Pin GPIO_PIN_6
#define CH340_RX_GPIO_Port GPIOD
#define Stepper_2_en2_Pin GPIO_PIN_9
#define Stepper_2_en2_GPIO_Port GPIOG
#define Stepper_2_dir2_Pin GPIO_PIN_10
#define Stepper_2_dir2_GPIO_Port GPIOG
#define Stepper_2_st2_Pin GPIO_PIN_11
#define Stepper_2_st2_GPIO_Port GPIOG
#define key1_Pin GPIO_PIN_12
#define key1_GPIO_Port GPIOG
#define key2_Pin GPIO_PIN_13
#define key2_GPIO_Port GPIOG
#define key3_Pin GPIO_PIN_14
#define key3_GPIO_Port GPIOG
#define key4_Pin GPIO_PIN_15
#define key4_GPIO_Port GPIOG
#define motor_frontA_input2_Pin GPIO_PIN_3
#define motor_frontA_input2_GPIO_Port GPIOB
#define oled_SCL_Pin GPIO_PIN_6
#define oled_SCL_GPIO_Port GPIOB
#define oled_SDA_Pin GPIO_PIN_7
#define oled_SDA_GPIO_Port GPIOB
#define motor_frontA_in2_Pin GPIO_PIN_8
#define motor_frontA_in2_GPIO_Port GPIOB
#define motor_frontA_in1_Pin GPIO_PIN_9
#define motor_frontA_in1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
