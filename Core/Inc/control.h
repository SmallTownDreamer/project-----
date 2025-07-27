#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f4xx_hal.h"

// JY901陀螺仪相关函数

void I2C_ScanDevice(void);

// 电机控制函数
void motor(int motorA, int motorB);
void limit(int *motorA, int *motorB);
int ABS(int v);

// 数据转换函数
short CharToShort(unsigned char cData[]);

// PID控制函数
int Motor_PI(int target_speed, int current_speed, int motor_id);
void Differential_Drive_Control(int forward_speed, int turn_speed);
int pid_run_wit(int16_t target, int16_t real, float kp, float ki, float kd);
int Angle_PID(float target_angle, float current_angle, float angular_velocity);
void Turn_To_Angle(float target_angle);
int ParseJY901Packet(unsigned char *packet);
int PIDUpdate(float target, float Act_Current, float Kp, float Ki, float Kd) ;
void motor_motion(int motorA,int motorB);

// 编码器读取函数
void read(void);
int read_speed(TIM_HandleTypeDef *htim);

// 小车主要状态枚举
typedef enum {
    CAR_STATE_WAIT_NUMBER = 0,      // 等待数字识别
    CAR_STATE_WAIT_MEDICINE,        // 等待放药
    CAR_STATE_DELAY_START,          // 延时
    CAR_STATE_FORWARD,              // 直行状态
    CAR_STATE_TURN_TO_ROOM,         // 转向病房
    CAR_STATE_ENTER_ROOM,           // 进入房间（等待）
    CAR_STATE_DELIVER_MEDICINE,     // 等待取药
    CAR_STATE_TURN_AROUND,          // 转身返回
    CAR_STATE_AUTO_RETURN,          // 自动返程
    CAR_STATE_WAIT_CROSSROAD,       // 等待交叉路口识别
    CAR_STATE_COMPLETED             // 任务完成
} CarState;

typedef enum {
    distanse_near=0,
    distanse_medium,
    distanse_remote
} Room;
#endif
