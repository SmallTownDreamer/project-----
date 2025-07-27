#include "stepper.h"
#include "main.h"
#include <math.h>
#include "control.h"

int stepper1_step = 0;  // 步进电机1步数计数
int stepper2_step = 0;  // 步进电机2步数计数
int stepper1_speed = 0; // 步进电机1速度
int stepper2_speed = 0; // 步进电机2速度

void stepper_init(void) // 步进电机初始化
{
    //  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); // en2
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); // step2
    stepper_move_step(1, 3200 * 1.57);                    // 步进电机1抬头初始化
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET); // en1
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // step1
}

void stepper_move_spst(int num, int speed, int steps) // 步进电机移动函数，speed控制速度大小，step控制方向,num=1_pitch,step<0抬头,num=2_yaw,
{
    switch (num) // 根据num选择步进电机
    {
    case 1:                                                 // 步进电机1
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); // en1
        if (steps > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); // dir1
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
        }
        stepper1_speed = speed;
        stepper1_speed = (stepper1_speed > 99) ? 99 : (stepper1_speed < -99) ? -99
                                                                             : stepper1_speed; // 步进电机1速度限幅
        break;

    case 2:                                                 // 步进电机2
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // en2
        if (steps > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // dir2
        }
        stepper2_speed = speed;
        stepper2_speed = (stepper2_speed > 99) ? 99 : (stepper2_speed < -99) ? -99
                                                                             : stepper2_speed; // 步进电机2速度限幅
        break;
    default:
        break;
    }
}

void stepper_move_speed(int num, int speed) // 步进电机移动函数,num=1_pitch,speed>0抬头,num=2_yaw,
{
    switch (num) // 根据num选择步进电机
    {
    case 1:                                                 // 步进电机1
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); // en1
        if (speed > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); // dir1
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
        }
        stepper1_speed = speed;
        stepper1_speed = (stepper1_speed > 99) ? 99 : (stepper1_speed < -99) ? -99
                                                                             : stepper1_speed; // 步进电机1速度限幅
        break;

    case 2:                                                 // 步进电机2
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // en2
        if (speed > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // dir2
        }
        stepper2_speed = speed;
        stepper2_speed = (stepper2_speed > 99) ? 99 : (stepper2_speed < -99) ? -99
                                                                             : stepper2_speed; // 步进电机2速度限幅
        break;
    default:
        break;
    }
}

void stepper_move_step(int num, int steps) // 步进电机移动函数,num=1_pitch,step>0抬头,num=2_yaw,
{
    int stepper_step = 0; // 步进电机步数
    switch (num)          // 根据num选择步进电机
    {
    case 1:                                                 // 步进电机1
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); // en1
        if (steps > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); // dir1
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
        }
        stepper_step = steps;
        break;

    case 2:                                                 // 步进电机2
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // en2
        if (steps > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // dir2
        }
        stepper_step = steps;
        break;
    default:
        break;
    }

    for (int i = 0; i < ABS(stepper_step) * 2; i++)
    {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2); // 控制步进电机步进
        // HAL_Delay(1);                          // 延时以控制步进速度
        uint32_t Delay = 50 * 168 / 4;
        do
        {
            __NOP();
        } while (Delay--);
    }
}
