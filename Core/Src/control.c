#include "control.h"
#include "main.h"
#include "oled.h"
#include "jy901s.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// 外部变量声明
extern uint8_t temp_ZFC[20];

// 目标角度
float target_angle;

// 计数
int cnt = 0;

// 速度限幅
#define speed_max 1000
#define speed_min -1000 // pwm最大值（限速可能还得小）
// 角度限幅
float angle_output_max = 500;
float angle_output_min = -500;

// 加速度
extern struct SAcc stcAcc;
// 角速度
extern struct SGyro stcGyro;
// 角度
extern struct SAngle stcAngle;

/************电机***************/
// 取绝对值函数
int ABS(int v)
{
    if (v > 0)
    {
        return v;
    }
    else
    {
        return -v;
    }
}
// 速度限制函数
void limit(int *motorA, int *motorB)
{
    if (*motorA > speed_max)
        *motorA = speed_max;
    if (*motorA < speed_min)
        *motorA = speed_min;
    if (*motorB > speed_max)
        *motorB = speed_max;
    if (*motorB < speed_min)
        *motorB = speed_min;
}
// 电机输入pwm(方向)
void motor(int motorA, int motorB)
{
    {
        limit(&motorA, &motorB);
        if (motorA > 0)
        {
            HAL_GPIO_WritePin(motor_frontA_in1_GPIO_Port, motor_frontA_in1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_frontA_in2_GPIO_Port, motor_frontA_in2_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(motor_frontA_in1_GPIO_Port, motor_frontA_in1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_frontA_in2_GPIO_Port, motor_frontA_in2_Pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, ABS(motorA));

        if (motorB > 0)
        {
            HAL_GPIO_WritePin(motor_frontB_in1_GPIO_Port, motor_frontB_in1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_frontB_in2_GPIO_Port, motor_frontB_in2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(motor_frontB_in1_GPIO_Port, motor_frontB_in1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_frontB_in2_GPIO_Port, motor_frontB_in2_Pin, GPIO_PIN_SET);
        }
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, ABS(motorB));
    }
}
// 电机读取电机编码器函数
uint8_t sys_tick = 0;
void read(void)
{
    if (uwTick - sys_tick < 10)
    {
        return;
    }
    uwTick = sys_tick;
    // A
    Encoder_Left = -read_speed(&htim2);
    // B
    Encoder_Right = read_speed(&htim8);
}
int read_speed(TIM_HandleTypeDef *htim)
{
    int temp;
    temp = (short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim, 0);
    return temp;
}
/*********陀螺仪******/
void jy901s_G()
{
    Gx = stcGyro.w[0] * 2000 / 32768;
    Gy = stcGyro.w[1] * 2000 / 32768;
    Gz = stcGyro.w[2] * 2000 / 32768;
}
void jy901s_Angle()
{
    Roll_x = stcAngle.Angle[0] * 180 / 32768;
    Pitch_y = stcAngle.Angle[1] * 180 / 32768;
    Yaw_z = stcAngle.Angle[2] * 180 / 32768;
}

/**************pid******** */
// 速度环******
//
// PI速度环（0左A，1右B）
int Motor_PI(int target_speed, int current_speed, int motor_id)
{
    static int err_last[2] = {0, 0}; // 两个电机的历史误差
    static float a = 0.5f;           // 滤波系数

    // 判断id合法性
    if (motor_id < 0 || motor_id > 1)
    {
        OLED_ShowString(0, 0, "speedpid err", 16);
        return 0;
    }

    // 计算误差
    int err = target_speed - current_speed;

    // 低通滤波
    int err_out = (int)((1 - a) * err + a * err_last[motor_id]);
    err_last[motor_id] = err_out;

    // 积分累积
    err_sum[motor_id] += err_out;

    // 积分限幅
    if (err_sum[motor_id] > 20000)
        err_sum[motor_id] = 20000;
    if (err_sum[motor_id] < -20000)
        err_sum[motor_id] = -20000;

    // PI控制计算
    int output = (int)(Velocity_Kp * err_out + Velocity_Ki * err_sum[motor_id]);

    return output;
}

// 差分驱动控制函数（目标前进速度 + 目标转向角度）
void Differential_Drive_Control(int forward_speed, int turn_speed)
{
    // 计算左右轮目标速度
    int target_left = forward_speed - turn_speed;  // 左轮
    int target_right = forward_speed + turn_speed; // 右轮

    // 分别进行速度闭环控制
    int output_left = Motor_PI(target_left, Encoder_Left, 0);
    int output_right = Motor_PI(target_right, Encoder_Right, 1);

    // 输出限幅
    limit(&output_left, &output_right);

    // 驱动电机
    motor(output_left, output_right);
}

// 角度PD控制全局变量
float angle_target = 0;

// 计算角度差值并归一化180度
float angle_difference(float target, float current)
{
    float diff = target - current;
    // 归一化到-180到180度范围
    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    else if (diff < -180.0f)
    {
        diff += 360.0f;
    }
    return diff;
}

// PD角度环（目标角度，当前角度，角速度）
int Angle_PID(float target_angle, float current_angle, float Gz)
{
    // 假设向右转为正
    //  计算角度误差（考虑圆周特性）
    float angle_error = 0; // 当前误差
    angle_error = angle_difference(target_angle, current_angle);

    // PD控制器计算：(极性未验证)
    float output = angle_Kp * angle_error - angle_Kd * Gz;

    // 输出限幅
    if (output > angle_output_max)
    {
        output = angle_output_max;
    }
    else if (output < angle_output_min)
    {
        output = angle_output_min;
    }

    return (int)output;
}

// 转向角度函数(输入目标转向的角度)
void Turn_To_Angle(float target_turn)
{
    // 陀螺仪测速

    // 转向start
    turn_state = 1;
    if (turn_state == 1)
    {
        target_angle = target_turn + Yaw_z;
        turn_state = 2;
    }
    if (turn_state == 2)
    {
        Angle_PID(target_angle, Yaw_z, Gz);
    }
}

// 电机函数
void motor_motion(int motorA, int motorB)
{
    read();
    int motor_A = Motor_PI(motorA, Encoder_Left, 0);
    int motor_B = Motor_PI(motorB, Encoder_Left, 1); // 待改
    motor(motor_A, motor_B);
}

// 总控制函数
void Control()
{

    // 0等待数字
    if (car_state == 0)
    {
        // 仅一次开启视觉接收中断
        if (temp_state == 0)
        {
            HAL_UART_Receive_IT(&huart4, recieve_buf, RECIEVE_SIZE);
            temp_state = 1;
        }
    }
    // 1等待放药（按钮替代）
    else if (car_state == 1)
    {
        if (HAL_GPIO_ReadPin(key4_GPIO_Port, key4_Pin) == 0)
        {
            cnt++;
        }
        else
        {
            cnt = 0;
        }
        if (cnt == 3)
        {
            car_state = 2;
            temp_state = 0;
        }
    }
    // 2等待一段时间(500ms)
    else if (car_state == 2)
    {
        // 仅一次初始化cnt
        if (temp_state == 0)
        {
            cnt = 0;
            temp_state = 1;
        }
        cnt++;
        if (cnt == 50)
        {
            temp_state = 0;
            car_state = 3;
            cnt = 0;
        }
    }
    // 3直行状态并等待车在交叉口的信息
    else if (car_state == 3)
    {
        motor_motion(500, 500);
        // 仅一次开启视觉接收中断
        if (temp_state == 0)
        {
            HAL_UART_Receive_IT(&huart4, recieve_buf, RECIEVE_SIZE);
            temp_state = 1;
        }
    }
    // 4近端病房进行转向
    else if (car_state == 4)
    {
        if (target_num == 1)
        {
            motor_motion(0, 500); // 可以改为角度环
        }
        else if (target_num == 2)
        {
            motor_motion(500, 0);
        }
        // 仅一次开启视觉接收中断
        if (temp_state == 0)
        {
            HAL_UART_Receive_IT(&huart4, recieve_buf, RECIEVE_SIZE);
            temp_state = 1;
        }
    }
    // 5驶向病房的循迹
    else if (car_state == 5)
    {
        motor_motion(500, 500);
        // 仅一次开启视觉接收中断
        if (temp_state == 0)
        {
            HAL_UART_Receive_IT(&huart4, recieve_buf, RECIEVE_SIZE);
            temp_state = 1;
        }
    }
    // 6停止并等待药品被拿走
    else if (car_state == 6)
    {
        motor_motion(0, 0);
        if (temp_state == 0)
            cnt = 0;
        if (HAL_GPIO_ReadPin(key4_GPIO_Port, key4_Pin) == 0)
        {
            cnt++;
        }
        else
        {
            cnt = 0;
        }
        if (cnt == 3)
        {
            car_state = 7;
            temp_state = 0;
        }
    }
    // 7等待一段时间(500ms)
    else if (car_state == 7)
    {
        // 仅一次初始化cnt
        if (temp_state == 0)
        {
            cnt = 0;
            temp_state = 1;
        }
        cnt++;
        if (cnt == 50)
        {
            temp_state = 0;
            car_state = 8;
            cnt = 0;
        }
    }
    // 8转弯回身
    else if (car_state == 8)
    {
        // 仅一次初始化cnt
        if (temp_state == 0)
        {
            cnt = 0;
            temp_state = 1;
        }
        cnt++;
        if (cnt == 50)
        {
            temp_state = 0;
            car_state = 8;
            cnt = 0;
        }
    }
    // 6在交叉口2处停止并识别数字
    else if (car_state == 5)
    {
        motor_motion(0, 0);
        if (temp_state == 0)
        {
            HAL_UART_Receive_IT(&huart4, recieve_buf, RECIEVE_SIZE);
            temp_state = 1;
        }
    }
}

// 增量式PID控制函数——经内部积分输出Output123123
int Out;
int PIDUpdate(float target, float Act_Current, float Kp, float Ki, float Kd)
{
    static int Err_Last = 0; // 上次、当前误差
    static int Err_Current = 0;
    static int Act_Last = 0; // 上次、当前实际值
    static int Output_sum = 0;
    const float k_VSI = 0, k_dif = 1;   // k_VSI——变速积分系数,越小限制越弱， k_dif——不完全微分系数,越大滤波越强
    Err_Last = Err_Current;             // 保存上次误差
    Act_Last = Act_Current;             // 保存上次实际值
    Err_Current = target - Act_Current; // 计算当前误差

    // 变速积分
    // Err_Current = (Err_Current > 0) ? Err_Current : (Err_Current < 0) ? -Err_Current
    //                                                                   : Err_Current; // 取Err_Current绝对值
    float C = 1 / (k_VSI * ABS(Err_Current) + 1); // 变速积分系数

    static int Dif_Last = 0;                                // 上次微分输出
    int Dif_Pre = -Kd * (Act_Current - Act_Last);           // 微分先行（防止目标值大幅度跳变时误差大幅度跳变输出过大的调控力）
    int Dif_out = (1 - k_dif) * Dif_Last + k_dif * Dif_Pre; // 不完全微分（低通滤波，减少外界噪声干扰）
    Dif_Last = Dif_out;                                     // 保存上次微分输出

    Out = Kp * (Err_Current - Err_Last) + C * Ki * Err_Current + Dif_out; // 增量式PID计算

    Output_sum += Out; // 累加输出
    Output_sum = (Output_sum > 800) ? 800 : (Output_sum < -800) ? -800
                                                                : Output_sum; // 输出限幅
    return Output_sum;                                                        // 返回输出值
}



// 陀螺仪读取函数(阻塞式，待改)
//  void JY901s(){
//      //临时数组
//      unsigned char chrTemp[24];

//     HAL_StatusTypeDef status;
//     uint8_t cnt=0;

//     status = HAL_I2C_Mem_Read(&hi2c2, (JY901_ADDR << 1), 0x34,
//                              I2C_MEMADD_SIZE_8BIT, chrTemp, 24, 1000);

//     if(status == HAL_OK)
//     {
//         // 加速度数据转换（变量名与显示一致）
//         Ax1 = (float)CharToShort(&chrTemp[0]) / 32768.0f * 16.0f;
//         Ay1 = (float)CharToShort(&chrTemp[2]) / 32768.0f * 16.0f;
//         Az1 = (float)CharToShort(&chrTemp[4]) / 32768.0f * 16.0f;

//         // 角速度数据转换
//         Gx1 = (float)CharToShort(&chrTemp[6]) / 32768.0f * 2000.0f;
//         Gy1 = (float)CharToShort(&chrTemp[8]) / 32768.0f * 2000.0f;
//         Gz1 = (float)CharToShort(&chrTemp[10]) / 32768.0f * 2000.0f;

//         // 磁场数据
//         Hx1 = CharToShort(&chrTemp[12]);
//         Hy1 = CharToShort(&chrTemp[14]);
//         Hz1 = CharToShort(&chrTemp[16]);

//         // 角度数据转换
//         Roll_x1 = (float)CharToShort(&chrTemp[18]) / 32768.0f * 180.0f;
//         Pitch_y1 = (float)CharToShort(&chrTemp[20]) / 32768.0f * 180.0f;
//         Yaw_z1 = (float)CharToShort(&chrTemp[22]) / 32768.0f * 180.0f;

//         // 显示成功状态
//         sprintf((char *)temp_ZFC, "JY901 OK");
//         OLED_ShowString(0, 56, temp_ZFC, 12);
//     }
//     else
//     {
//         // 显示I2C错误状态
//         sprintf((char *)temp_ZFC, "I2C Error: %d", status);
//         OLED_ShowString(0, 56, temp_ZFC, 12);

//         // 将所有数据设为0
//         Ax1 = Ay1 = Az1 = 0.0f;
//         Gx1 = Gy1 = Gz1 = 0.0f;
//         Hx1 = Hy1 = Hz1 = 0;
//         Roll_x1 = Pitch_y1 = Yaw_z1 = 0.0f;
//     }

//     // 显示传感器数据
//     // sprintf((char *)temp_ZFC, "AX:%.2f  Gx:%.2f", Ax1, Gx1);
//     // OLED_ShowString(0, 0, temp_ZFC, 12);
//     // sprintf((char *)temp_ZFC, "roll_x:%.2f", Roll_x1);
//     // OLED_ShowString(0, 16, temp_ZFC, 12);
//     // sprintf((char *)temp_ZFC, "y:%.2f", Pitch_y1);
//     // OLED_ShowString(0, 32, temp_ZFC, 12);
//     // sprintf((char *)temp_ZFC, "z:%.2f", Yaw_z1);
//     // OLED_ShowString(0, 48, temp_ZFC, 12);
//     // else {cnt++;
//     // do{
//     //     status = HAL_I2C_Mem_Read(&hi2c2, (JY901_ADDR<<1), 0x34,I2C_MEMADD_SIZE_8BIT, chrTemp, 24, 1000);
//     // }while (cnt==3||status == HAL_OK );
//     // if(cnt==3){
//     //     OLED_ShowString(0,0,"iic error",9);}
//     // }
// }
// void JY901s(){
//     //临时数组
//     unsigned char chrTemp[24];

//     HAL_StatusTypeDef status;
//     uint8_t cnt=0;

//     if(status == HAL_OK)
//     {
//         // 加速度数据转换（变量名与显示一致）
//         Ax1 = (float)CharToShort(&chrTemp[0]) / 32768.0f * 16.0f;
//         Ay1 = (float)CharToShort(&chrTemp[2]) / 32768.0f * 16.0f;
//         Az1 = (float)CharToShort(&chrTemp[4]) / 32768.0f * 16.0f;

//         // 角速度数据转换
//         Gx1 = (float)CharToShort(&chrTemp[6]) / 32768.0f * 2000.0f;
//         Gy1 = (float)CharToShort(&chrTemp[8]) / 32768.0f * 2000.0f;
//         Gz1 = (float)CharToShort(&chrTemp[10]) / 32768.0f * 2000.0f;

//         // 磁场数据
//         Hx1 = CharToShort(&chrTemp[12]);
//         Hy1 = CharToShort(&chrTemp[14]);
//         Hz1 = CharToShort(&chrTemp[16]);

//         // 角度数据转换
//         Roll_x1 = (float)CharToShort(&chrTemp[18]) / 32768.0f * 180.0f;
//         Pitch_y1 = (float)CharToShort(&chrTemp[20]) / 32768.0f * 180.0f;
//         Yaw_z1 = (float)CharToShort(&chrTemp[22]) / 32768.0f * 180.0f;

//         // 显示成功状态
//         sprintf((char *)temp_ZFC, "JY901 OK");
//         OLED_ShowString(0, 56, temp_ZFC, 12);
//     }
//     else
//     {
//         // 显示I2C错误状态
//         sprintf((char *)temp_ZFC, "I2C Error: %d", status);
//         OLED_ShowString(0, 56, temp_ZFC, 12);

//         // 将所有数据设为0
//         Ax1 = Ay1 = Az1 = 0.0f;
//         Gx1 = Gy1 = Gz1 = 0.0f;
//         Hx1 = Hy1 = Hz1 = 0;
//         Roll_x1 = Pitch_y1 = Yaw_z1 = 0.0f;
//     }

// }
