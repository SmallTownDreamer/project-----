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
// 视觉接收数组
uint8_t RxCamera_buf[16];
// 视觉发送数组
uint8_t TxCamera_buf[6];

// 临时字符串
uint8_t temp_ZFC[20];

// 步进电机
extern int steppery_st; // 步进电机1初始步数
extern int stepperx_st; // 步进电机2初始步数
uint8_t test_if1 = 0, test_if2 = 0;

// 视觉收发相关
uint8_t uart_rx_byte;     // 单字节接收缓冲
uint8_t rx_index = 0;     // 接收索引
uint8_t packet_ready = 0; // 数据包准备标志
// 视觉接收数组
uint8_t carecieve_buf[16], rxprocess_buf[20];
// 视觉最终数组
int16_t camera_use[10];

// 陀螺仪接收参数
extern uint8_t Rxdata;
extern int m1, m2;
// 角度（自动解算）
float Roll_x, Pitch_y, Yaw_z;
// 角加速度
float Ax, Ay, Az;
// 角速度
float Gx, Gy, Gz;
// 模式选择
uint8_t menu;
// PID参数
float Kp = 0.5, Kd = 0.0, Ki = 0;

extern int steppery_speed; // 步进电机1速度
extern int stepperx_speed; // 步进电机2速度
uint8_t pid_flag = 0;
uint32_t nnn;

// 模式选择
uint8_t current_mode = 0; // 当前选中的模式：0=未选，1=模式1，2=模式2
// 子选项选择（仅模式1有效）
uint8_t current_sub_option = 0; // 当前选中的子选项：0=未选，1=Yaw，2=Dir_y，3=Pitch
// 子选项可选范围
const uint8_t YAW_MIN = 1, YAW_MAX = 4;
const uint8_t DIR_Y_MIN = 1, DIR_Y_MAX = 2;
const uint8_t PITCH_MIN = 1, PITCH_MAX = 3;
// 当前选中的选项的数值（由 state 控制）
uint8_t state = 1; // 当前被选中选项的数值，比如 Yaw = 3
// 是否正在运行
uint8_t is_running = 0; // 0 = 停止，1 = 运行
// 按键标志位（在中断里设置）
extern volatile uint8_t key1_flag; // key1：state 减1
extern volatile uint8_t key3_flag; // key3：state 加1
extern volatile uint8_t key2_flag; // key2：开始/停止
extern volatile uint8_t key4_flag; // key4：切换子选项 或 模式
uint8_t Clamp(uint8_t value, uint8_t min, uint8_t max);
uint8_t GetMinValue();
uint8_t GetMaxValue();
void Run_Current_Mode_Function();

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

  // 看我！！
  // 我把所有之前用12命名步进的地方变为了xy(见stepper.h)，包括后面的pid的传入参数也是xy注意一下
  // 陀螺仪接收中断
  HAL_UART_Receive_IT(&huart3, &Rxdata, 1);
  // 下面while（1）里有一个测试的蓝牙发送代码取注释掉看看串口，（如果是好的）把while（1）上面的那个代码取注释掉看看定义试试

  // 开启步进电机定时器中断
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim8);

  // 步进电机初始化
  stepper_init(); // 步进电机初始化
                  // 蓝牙调参
  HAL_UART_Receive_IT(&huart1, recieve_buf, 5);

  // 视觉接收中断
  HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);
  nnn = uwTick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    // key1：当前选项 state 减 1
    if (key1_flag)
    {
      if (state > GetMinValue())
        state--;
      key1_flag = 0;
    }

    // key3：当前选项 state 加 1
    if (key3_flag)
    {
      if (state < GetMaxValue())
        state++;
      key3_flag = 0;
    }

    // key2：切换 is_running 状态（启动/停止）
    if (key2_flag)
    {
      is_running = !is_running;
      key2_flag = 0;
    }

    // key4：切换子选项（Yaw/Dir_y/Pitch） 或 切换模式
    if (key4_flag)
    {
      if (current_mode == 1)
      {
        // 模式1下：切换子选项 Yaw -> Dir_y -> Pitch -> Yaw...
        current_sub_option++;
        if (current_sub_option > 3)
          current_sub_option = 1;
      }
      else
      {
        // 模式2 或其它模式下：可以切换模式，比如 current_mode = 1 / 2
        current_mode++;
        if (current_mode > 2)
          current_mode = 1;
      }
      key4_flag = 0;
    }

    // --------------------------
    // 当前选项范围限制
    // --------------------------
    state = Clamp(state, GetMinValue(), GetMaxValue());

    // --------------------------
    // 如果当前模式正在运行，执行功能
    // --------------------------
    if (is_running)
    {
      Run_Current_Mode_Function();
    }

    cam_receive();
    if (!pid_flag)
    {
      if (camera_use[0] == 0x00)
      {
        stepperx_speed = 100;
        steppery_speed = 0;
      }
      if (camera_use[0] == 0x01)
      {
        stepperx_speed = 0;
        steppery_speed = 0;
      }
      if (camera_use[0] == 0x02)
      {
        pid_flag = 1;
        stepperx_speed = PIDUpdate(0, camera_use[1], x);
        steppery_speed = PIDUpdate(0, camera_use[2], y);
      }
      stepper_move_speed(y, steppery_speed);
      stepper_move_speed(x, -stepperx_speed);
    }
    else
    {
      stepperx_speed = PIDUpdate(0, camera_use[1], x);
      steppery_speed = PIDUpdate(0, camera_use[2], y);
      stepper_move_speed(y, steppery_speed / 20);
      stepper_move_speed(x, -stepperx_speed);
    }
    if (uwTick - nnn > 4000)
    {
      uint8_t buf[] = {0X6A, 0X00, 0X00, 0X00, 0X6B, 0X5A};
      HAL_UART_Transmit(&huart4, buf, 6, 1000);
    }
    printf(" %f, %f, %f,%d,%d,%d,%d\n", Kp, Kd, Ki, camera_use[1], stepperx_speed, camera_use[2], steppery_speed);
    // menu = mode_choose();
    // if (menu == 1)
    // {
    //   mode1();
    // }
    // if (menu == 2)
    // {
    //   mode2();
    // }
    // if (menu == 3)
    // {
    //   mode3();
    // }

    // 激光测试
    //  LASER_ON();
    //  HAL_Delay(1000);
    //  LASER_OFF();
    //  HAL_Delay(1000);

    // 陀螺仪调试
    //  sprintf((char *)temp_ZFC, "angle:%d   ", stcAngle.Angle[0]*180/32768 );
    //  OLED_ShowString(0, 0, temp_ZFC, 16);
    //  printf("%d\n",stcAngle.Angle[0]*180/32768);
    //  HAL_Delay(100);

    // 步进电机调参
    // if (stepperx_st >= 1600)
    // {

    // }
    // else if (stepperx_st <= -1600)
    // {
    // stepper_move_speed(x, 40);
    // }

    // float t = (HAL_GetTick() % 6283) / 1000.0f; // 6283约等于2π*1000
    // int temp1 = 40 * sin(t);//画圆
    // int temp2 = 40 * cos(t);//画圆
    // 蓝牙速度控制
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

// 摄像头处理函数
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
    //        carecieve_buf[0], carecieve_buf[1], carecieve_buf[2], carecieve_buf[3],
    //        carecieve_buf[4], carecieve_buf[5], carecieve_buf[6], carecieve_buf[7],
    //        carecieve_buf[8], carecieve_buf[9], carecieve_buf[10], carecieve_buf[11],
    //        carecieve_buf[12], carecieve_buf[13], carecieve_buf[14], carecieve_buf[15], carecieve_buf[16]);
    cam_process();
  }
}
// 视觉处理
void cam_process(void)
{
  camera_use[0] = (uint8_t)rxprocess_buf[0];                                                // 0未找到|1找到未确定|2确认找到
  camera_use[1] = (int16_t)((uint8_t)rxprocess_buf[1]) | (uint16_t)(rxprocess_buf[2] << 8); // dx
  camera_use[2] = (int16_t)((uint8_t)rxprocess_buf[3]) | (uint16_t)(rxprocess_buf[4] << 8); // dy
  camera_use[3] = (int16_t)((uint8_t)rxprocess_buf[1]) | (uint16_t)(rxprocess_buf[6] << 8); // pitch
  camera_use[4] = (int16_t)((uint8_t)rxprocess_buf[3]) | (uint16_t)(rxprocess_buf[8] << 8); // yaw
}

// 模式选择相关函数
uint8_t Clamp(uint8_t value, uint8_t min, uint8_t max) //  限制 value 在 [min, max] 范围内，超出则截断
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}
uint8_t GetMinValue()// 根据当前选中的子选项返回最小值
{
  switch (current_sub_option)
  {
  case 1:
    return YAW_MIN; // Yaw: 1~4
  case 2:
    return DIR_Y_MIN; // Dir_y: 1~2
  case 3:
    return PITCH_MIN; // Pitch: 1~3
  default:
    return 1; // 默认最小值
  }
}
uint8_t GetMaxValue()// 根据当前选中的子选项返回最大值
{
  switch (current_sub_option)
  {
  case 1:
    return YAW_MAX; // Yaw: 1~4
  case 2:
    return DIR_Y_MAX; // Dir_y: 1~2
  case 3:
    return PITCH_MAX; // Pitch: 1~3
  default:
    return 1; // 默认最大值
  }
}
void Run_Current_Mode_Function() //根据当前模式和选项执行功能
{
  if (current_mode == 1)
  {
    // 模式 1：根据当前选中的子选项执行对应功能
    switch (current_sub_option)
    {
    case 1:
      // 执行 Yaw 相关功能，比如控制电机角度、显示等
      // 示例：printf("Mode 1 - Yaw: %d\n", state);
      break;
    case 2:
      // 执行 Dir_y 相关功能
      // printf("Mode 1 - Dir_y: %d\n", state);
      break;
    case 3:
      // 执行 Pitch 相关功能
      // printf("Mode 1 - Pitch: %d\n", state);
      break;
    }
  }
  else if (current_mode == 2)
  {
    // 模式 2：无子选项，执行模式2的功能
    // printf("Mode 2 is running\n");
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
