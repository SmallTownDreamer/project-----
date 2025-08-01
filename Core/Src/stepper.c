#include "stepper.h"
#include "main.h"
#include <math.h>
#include "control.h"
#include "tim.h"

int steppery_step = 0;  // 步进电机1步数计数
int stepperx_step = 0;  // 步进电机2步数计数
int steppery_speed = 0; // 步进电机1速度
int stepperx_speed = 0; // 步进电机2速度
int steppery_st = 0;   // 步进电机1初始步数
int stepperx_st = 0;   // 步进电机2初始步数



void stepper_init(void) // 步进电机初始化
{
    // stepper_move_step(x, 3200 * 10);
    // //  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); // en2
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); // step2
    stepper_move_step(y, 3200 * 1.7);                     // 步进电机1抬头初始化
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET); // en1
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // step1
    steppery_st = 0;                                     // 步进电机1初始步数
    stepperx_st = 0;                                     // 步进电机2初始步数
    // stepper_move_speed(x, 20);
}

void stepper_move_speed(stepper_num num, int speed) // 步进电机移动函数,num=1_pitch,speed>0抬头,num=2_yaw,
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
        // __HAL_TIM_SET_AUTORELOAD(&htim5, 350 - ABS(speed));
         __HAL_TIM_SET_AUTORELOAD(&htim7, ABS((int)(20000/speed))); // 设置步进电机定时器自动重装载值
        if (speed == 0)
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // step1
        }
        steppery_speed = speed;
        steppery_speed = (steppery_speed > 100) ? 100 : (steppery_speed < -100) ? -100
                                                                                : steppery_speed; // 步进电机1速度限幅
        break;

    case 2:                                                 // 步进电机2
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // en2
        if (speed > 0)                                      // 顺时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // dir2
        }
        else // 逆时针
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4,GPIO_PIN_SET); // dir2
        }
        // __HAL_TIM_SET_AUTORELOAD(&htim6, 350 - ABS(speed)); // 设置步进电机定时器自动重装载值
        __HAL_TIM_SET_AUTORELOAD(&htim6, ABS((int)(10000/speed)));
        if (speed == 0)
        {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); // step2
        }
        stepperx_speed = speed;
        stepperx_speed = (stepperx_speed > 100) ? 100 : (stepperx_speed < -100) ? -100
                                                                                : stepperx_speed; // 步进电机2速度限幅
        break;
    default:
        break;
    }
}

void stepper_move_step(stepper_num num, int steps) // 步进电机移动函数,num=1_pitch,step>0抬头,num=2_yaw,
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
        switch (num)
        {
        case 1:
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2); // 控制步进电机1步进
            break;
        case 2:
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5); // 控制步进电机2步进
        default:
            break;
        }
        // HAL_Delay(1);                          // 延时以控制步进速度
        uint32_t Delay = 50 * 168 / 4;
        do
        {
            __NOP();
        } while (Delay--);
    }
}

void stepper_goto_position(int x_target, int y_target)
{
    // 计算需要移动的步数
    int dx = x_target - stepperx_st;
    int dy = y_target - steppery_st;

    // 控制X轴步进电机
    if (dx != 0)
    {
        stepper_move_step(x, dx); // x轴移动dx步
        stepperx_st = x_target;   // 更新当前位置
    }

    // 控制Y轴步进电机
    if (dy != 0)
    {
        stepper_move_step(y, dy); // y轴移动dy步
        steppery_st = y_target;   // 更新当前位置
    }
}

int PIDUpdate(float target, float Act_Current, stepper_num motor_id) 
{
    // 为两个电机分别保存PID状态
    static int Err_Last[3] = {0, 0}; // 上次误差
    static int Err_Current[3] = {0, 0};
    static int Act_Last[3] = {0, 0}; // 上次实际值
    static int Output_sum[3] = {0, 0};
    static int Out[3] = {0, 0,0};
    static int Dif_Last[3] = {0, 0,0};
    const float k_VSI=0, k_dif=1;
    const float Kp,Kd,Ki;

    Err_Last[motor_id] = Err_Current[motor_id];        // 保存上次误差
    Act_Last[motor_id] = Act_Current;                  // 保存上次实际值
    Err_Current[motor_id] = target - Act_Current;      // 计算当前误差

    float C = 1 / (k_VSI * ABS(Err_Current[motor_id]) + 1); // 变速积分系数

    int Dif_Pre = -Kd * (Act_Current - Act_Last[motor_id]);
    int Dif_out = (1 - k_dif) * Dif_Last[motor_id] + k_dif * Dif_Pre;
    Dif_Last[motor_id] = Dif_out;

    Out[motor_id] = Kp * (Err_Current[motor_id] - Err_Last[motor_id]) + C * Ki * Err_Current[motor_id] + Dif_out;

    Output_sum[motor_id] += Out[motor_id];
    if(Output_sum[motor_id]>800){Output_sum[motor_id]=800;}
    if(Output_sum[motor_id]<-800){Output_sum[motor_id]=-800;}
    return Output_sum[motor_id];
}

// void stepper_move_spst(int num, int speed, int steps) // 步进电机移动函数，speed控制速度大小，step控制方向,num=1_pitch,step<0抬头,num=2_yaw,
// {
//     switch (num) // 根据num选择步进电机
//     {
//     case 1:                                                 // 步进电机1
//         HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); // en1
//         if (steps > 0)                                      // 顺时针
//         {
//             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); // dir1
//         }
//         else // 逆时针
//         {
//             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); // dir1
//         }
//         steppery_speed = speed;
//         steppery_speed = (steppery_speed > 99) ? 99 : (steppery_speed < -99) ? -99
//                                                                              : steppery_speed; // 步进电机1速度限幅
//         break;

//     case 2:                                                 // 步进电机2
//         HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); // en2
//         if (steps > 0)                                      // 顺时针
//         {
//             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET); // dir2
//         }
//         else // 逆时针
//         {
//             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET); // dir2
//         }
//         stepperx_speed = speed;
//         stepperx_speed = (stepperx_speed > 99) ? 99 : (stepperx_speed < -99) ? -99
//                                                                              : stepperx_speed; // 步进电机2速度限幅
//         break;
//     default:
//         break;
//     }
// }
