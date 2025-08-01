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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include "control.h"
#include "track.h"
#include "table.h"
#include "stepper.h"
#include "math.h"
#include "string.h"
#include "table.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 蓝牙接收数组
uint8_t recieve_buf[5];
// 蓝牙发送数组
uint8_t transmit_buf[20];
//视觉接收数组
uint8_t RxCamera_buf[16];
//视觉发送数组
uint8_t TxCamera_buf[6];

// 临时字符串
uint8_t temp_ZFC[20];

// 步进电机
extern int steppery_st; // 步进电机1初始步数
extern int stepperx_st; // 步进电机2初始步数
uint8_t test_if1=0,test_if2=0;

//视觉收发相关
uint8_t uart_rx_byte;           // 单字节接收缓冲
uint8_t rx_index = 0;    // 接收索引
uint8_t packet_ready = 0; // 数据包准备标志
// 视觉接收数组
uint8_t carecieve_buf[16],rxprocess_buf[20];
//视觉最终数组
int16_t camera_use[10];

//陀螺仪接收参数
extern uint8_t Rxdata;
extern int m1, m2;
// 角度（自动解算）
float Roll_x, Pitch_y, Yaw_z;
// 角加速度
float Ax, Ay, Az;
// 角速度
float Gx, Gy, Gz;
//模式选择
uint8_t menu;
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
int16_t tick;

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
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  // OLED_Init();
  // OLED_Clear();
  // OLED_ShowString(0, 0, "oled ready", 16);

//看我！！
//我把所有之前用12命名步进的地方变为了xy(见stepper.h)，包括后面的pid的传入参数也是xy注意一下
  //陀螺仪接收中断
  HAL_UART_Receive_IT(&huart3,&Rxdata,1);
  //下面while（1）里有一个测试的蓝牙发送代码取注释掉看看串口，（如果是好的）把while（1）上面的那个代码取注释掉看看定义试试

  // 开启步进电机定时器中断
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim8);
  
  // 步进电机初始化
  stepper_init(); // 步进电机初始化

  //蓝牙调参
  HAL_UART_Receive_IT(&huart1,recieve_buf,5);

  // 视觉接收中断
  HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    menu=mode_choose();
		if(menu ==1){mode1();}
		if(menu ==2){mode2();}
		if(menu ==3){mode3();}



    //激光测试
    // LASER_ON();
    // HAL_Delay(1000);
    // LASER_OFF();
    // HAL_Delay(1000);
    
    //陀螺仪调试
    // sprintf((char *)temp_ZFC, "angle:%d   ", stcAngle.Angle[0]*180/32768 );
    // OLED_ShowString(0, 0, temp_ZFC, 16);
    // printf("%d\n",stcAngle.Angle[0]*180/32768);
    // HAL_Delay(100);

    // 步进电机调参
    // if (stepperx_st >= 1600)
    // {
    //   stepper_move_speed(x, -40);
    // }
    // else if (stepperx_st <= -1600)
    // {
    //   stepper_move_speed(x, 40);
    // }

// float t = (HAL_GetTick() % 6283) / 1000.0f; // 6283约等于2π*1000
// int temp1 = 40 * sin(t);//画圆
// int temp2 = 40 * cos(t);//画圆
//蓝牙速度控制
//     stepper_move_speed(x,m1);//齿轮比为10：100
//     stepper_move_speed(y,m2);//齿轮比为10：50（控速函数我的写法是同样的数值输入到speed函数中实际转速相同，可以用vofa试试速度（电脑密码989898））
    // printf("%d,%d,%d,%d\n", steppery_st, stepperx_st,m1,m2);
    cam_receive();
    // printf("%d,%d,%d,%d,%d,%d\n",rxprocess_buf[0],rxprocess_buf[1],rxprocess_buf[2],camera_use[0],camera_use[1],camera_use[2]);
    // cam_process();         
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
// 字节转浮点数函数
float bytes_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  unsigned char data[] = {b0, b1, b2, b3};
  return *(float *)data;
}

//摄像头处理函数
void cam_receive(void)
{
  // 在主循环中处理接收到的完整数据包
  if (packet_ready)
  {
    packet_ready = 0;

    // 验证数据包
    uint32_t sum = 0;
    for (int i = 0; i < 14; i++)
    {
      sum += carecieve_buf[i];
    }
    sum = sum & 0xff;

    // // 显示调试信息
    // OLED_ShowNum(0, 0, recieve_buf[0], 5, 16);
    // OLED_ShowNum(0, 2, recieve_buf[1], 5, 16);
    // OLED_ShowNum(0, 4, recieve_buf[2], 5, 16);
    // OLED_ShowNum(0, 6, recieve_buf[3], 5, 16);
    // OLED_ShowNum(64, 0, recieve_buf[4], 5, 16);
    // OLED_ShowNum(64, 2, recieve_buf[5], 5, 16);
    // OLED_ShowNum(64, 4, recieve_buf[14], 5, 16);
    // OLED_ShowNum(64, 6, recieve_buf[15], 5, 16);

    // 验证帧头、帧尾和校验和
    if (carecieve_buf[0] == 0x6A && carecieve_buf[15] == 0x5A && carecieve_buf[14] == sum)
    {
      // 数据包有效，复制到处理缓冲区
      memcpy(&rxprocess_buf[0], &carecieve_buf[1], 13);
    }
    else
    {
      // printf("Invalid packet: head=%02X, tail=%02X, checksum=%02X, calc=%02X\n",
      //        carecieve_buf[0], carecieve_buf[15], carecieve_buf[14], (uint8_t)sum);
    }

      // printf("%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
      //    carecieve_buf[0], carecieve_buf[1], carecieve_buf[2], carecieve_buf[3],
      //    carecieve_buf[4], carecieve_buf[5], carecieve_buf[6], carecieve_buf[7],
      //    carecieve_buf[8], carecieve_buf[9], carecieve_buf[10], carecieve_buf[11],
      //    carecieve_buf[12], carecieve_buf[13], carecieve_buf[14], carecieve_buf[15], carecieve_buf[16]);
      cam_process();
        }
}
//视觉处理
void cam_process(void){
  camera_use[0]=(uint8_t)rxprocess_buf[0];
  camera_use[1]=(int16_t)((uint8_t)rxprocess_buf[1])|(uint16_t)(rxprocess_buf[2]<<8);
  camera_use[2]=(int16_t)((uint8_t)rxprocess_buf[3])|(uint16_t)(rxprocess_buf[4]<<8);  


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
