#ifndef _STEPPER_H_
#define _STEPPER_H_

void stepper_init(void);                     // 步进电机初始化
void stepper_move_step(int num, int steps);  // 步进电机移动函数
void stepper_move_speed(int num, int speed); // 步进电机速度设置函数
// void stepper_move_spst(int num, int speed, int steps); //// 步进电机移动函数，speed控制速度大小，step控制方向,num=1_pitch,step<0抬头,num=2_yaw,

#endif
