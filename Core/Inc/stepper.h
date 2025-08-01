#ifndef _STEPPER_H_
#define _STEPPER_H_

typedef enum{
    y=1,
    x
} stepper_num; //y为1，x为2（如果不对记得改）

void stepper_init(void);                     // 步进电机初始化
void stepper_move_step(stepper_num num, int steps) ;// 步进电机移动函数
void stepper_move_speed(stepper_num num, int speed);// 步进电机速度设置函数
// void stepper_move_spst(int num, int speed, int steps); //// 步进电机移动函数，speed控制速度大小，step控制方向,num=1_pitch,step<0抬头,num=2_yaw,
void stepper_goto_position(int x_target, int y_target);
int PIDUpdate(float target, float Act_Current, stepper_num motor_id); 

#endif
