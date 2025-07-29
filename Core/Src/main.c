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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include "control.h"
#include "track.h"
#include "table.h"
#include "jy901s.h"
#include "stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 蓝牙接收数组
uint8_t recieve_buf[RECIEVE_SIZE];
// 蓝牙发送数组
uint8_t transmit_buf[TRANSMIT_SIZE];
// 临时字符串
uint8_t temp_ZFC[20];

// 外部声明：双缓冲区变量
extern uint8_t process_buf[11];
extern volatile uint8_t data_ready;

// 模式选择
uint8_t menu;

// 角加速度
float Ax, Ay, Az;
// 角速度
float Gx, Gy, Gz;
// 磁场
short Hx, Hy, Hz;
// 角度（自动解算）
float Roll_x, Pitch_y, Yaw_z;

// 电机测速(读速函数待区分左右)
signed int Encoder_Left, Encoder_Right;

// 目标数字和识别数字
uint8_t target_num, now_num[4], store_num2[2], store_num31[2], store_num32[2];
// 临时状态
uint8_t temp_state = 0;

// 转向状态量(0off,1start,2continue)
uint8_t turn_state = 0;

// 送药小车状态
CarState car_state = 0;

// err状态
uint8_t err_state = 0;

// 速度环
float Velocity_Ki = 0, Velocity_Kp = 0, Velocity_Kd = 0;
// 角度环
float angle_Kp = 15.0, angle_Kd = 8.0;

// 目标速度
int target_V;
// 目标角度
float target_turn;

// 路径记录
uint8_t Path[5];
// 数字存储
Room store_num[9];

int err_sum[2];

// 加速度
extern struct SAcc stcAcc;
// 角速度
extern struct SGyro stcGyro;
// 角度
extern struct SAngle stcAngle;
// 陀螺仪接收参数
extern uint8_t Rxdata;

// 步进电机
extern int stepper1_st;  // 步进电机1步数_内部
extern int stepper2_st;  // 步进电机2步数_内部
extern int stepper1_0st; // 步进电机1初始步数
extern int stepper2_0st; // 步进电机2初始步数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "oled ready", 16);

  // 电机初始化
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  motor(0, 0);
  // OLED_ShowString(0, 0, "motor aready", 16);

  // 开启定时器中断
  HAL_TIM_Base_Start_IT(&htim5);

  // // 步进电机初始化

  stepper_init(); // 步进电机初始化
  // stepper_move_speed(2, -20);
  // stepper_move_speed(1, -10);
  // stepper_move_step(1, -5024);
  // stepper_move_speed(1, 10);
  // __HAL_TIM_SET_AUTORELOAD(&htim5, 350);
  // stepper_move_step(2, 4960);

  // 陀螺仪接收中断
  // HAL_UART_Receive_IT(&huart2, &Rxdata, 1);

  // // 蓝牙接收中断
  // HAL_UART_Transmit(&huart1, transmit_buf, TRANSMIT_SIZE, 1000);

  // 陀螺仪

  // //调参蓝牙中断
  // HAL_UART_Receive_IT(&huart1, recieve_buf, 5);
  // // 蓝牙测试
  // bluetooth_test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_time = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // // 增量式pid调参
    // int temp;
    // read();
    // temp = PIDUpdate(target_V, Encoder_Left, Velocity_Kp, Velocity_Ki, Velocity_Kd);
    // printf("%d,%d,%d,%f,%f,%f\n", Encoder_Left, target_V, temp, Velocity_Ki, Velocity_Kp, Velocity_Kd);
    // motor(temp, temp);
    // OLED_ShowNum(0, 0, Out, 3, 16);

    // 陀螺仪调试
    // sprintf((char *)temp_ZFC, "%d   ", stcAngle.Angle[0] * 180 / 32768);
    // OLED_ShowString(0, 0, temp_ZFC, 16);

    // 步进电机调参
    if (stepper2_0st >= 400)
    {
      stepper_move_speed(2, -20);
    }
    else if (stepper2_0st <= -400)
    {
      stepper_move_speed(2, 20);
    }
    // if (stepper1_0st >= 200)
    // {
    //   stepper_move_speed(1, -20);
    // }
    // else if (stepper1_0st <= -200)
    // {
    //   stepper_move_speed(1, 20);
    // }
    // stepper_move_speed(1, 10);
    float temp1 = sin(stepper2_0st / 100 * 3.14);
    int temp2 = round(temp1);
    stepper_move_speed(1, temp2);

    OLED_ShowNum(0, 4, stepper2_0st, 5, 16);
    OLED_ShowNum(0, 6, 1, 5, 16);
    OLED_ShowNum(0, 6, 0, 5, 16);

    // 角度环调参
    int temp;
    temp = Angle_PID(target_turn, Yaw_z, Gz);
    printf("%d,%d,%d,%f,%f,%f,%f,%f,%d\n", 1, 1, 1, 1.0, 1.0, 1.0, target_turn, Yaw_z, temp);
    motor_motion(-temp, temp);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
// 蓝牙测试
void bluetooth_test()
{
  // 车A
  uint8_t test_buf[] = {0xA5, 0x11, 0x22, 0x5A};
  HAL_UART_Transmit(&huart1, test_buf, 4, 1000);
  HAL_UART_Receive_IT(&huart1, recieve_buf, 4);
  OLED_ShowString(0, 2, "test BT_transmit...", 16);
  HAL_Delay(400);
  // //车B
  // HAL_UART_Receive_IT(&huart1,recieve_buf,4);
  // OLED_ShowString(0,2,"test BT_transmit...",16);
  // HAL_Delay(200);
  // 总
  if (bt_state == 1)
  {
    OLED_ShowString(0, 4, "BT_transmit suceess!", 16);
  }
  else
  {
    OLED_ShowString(0, 4, "BT_transmit fail QAQ", 16);
  }
}

// 串口重定向
#ifdef __GNUC__
// GCC编译器版本（STM32CubeIDE使用）
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
// Keil MDK-ARM编译器版本
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
  return ch;
}

#endif

// 错误显示函数
void err_out()
{
  switch (err_state)
  {
  case 1:
    OLED_ShowString(0, 0, "camera err", 16);
    break;

  default:
    break;
  }
}

// 字节转浮点数函数
float bytes_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  unsigned char data[] = {b0, b1, b2, b3};
  return *(float *)data;
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
