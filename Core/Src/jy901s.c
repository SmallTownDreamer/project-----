#include "jy901s.h"
#include <string.h>
#include "usart.h"
#include "stepper.h"
#include "table.h"
float angle_difference(float target, float current);
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

char MAGNETICCALAM[5] = {0XFF,0XAA,0X01,0X07,0X00};
char SAVEMAGNETICCALAM[5] = {0XFF,0XAA,0X00,0X00,0X00};

//用串口3给JY模块发送指令
// void sendcmd(char cmd[])
// {
// 	char i;
// 	for(i=0;i<5;i++)
// 		UART3_send_char(cmd[i]);
// }

// extern void OLED_ShowNum(uint8_t x, uint8_t y, uint16_t num, uint8_t len, uint8_t size);
uint8_t ucRxBuffer_1[256];
uint8_t ucRxCount_1 = 0;	//接收数据计数



void uart3_read_data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[256];
	static unsigned char ucRxCount = 0;	
	ucRxCount_1=ucRxCount;	//将接收计数赋值给另一个变量
	ucRxBuffer_1[ucRxCount]=ucData;	//将收到的数据存入缓冲区中
	ucRxBuffer[ucRxCount++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) 				//数据头
	{
		ucRxCount=0;
		return;
	}
	if (ucRxCount<11) {return;}			//数据不满11个，则返回
	else
	{
		// 校验和计算
		unsigned char sum = 0;
		for (int i = 0; i < 10; i++) {
			sum += ucRxBuffer[i];
		}
		if (sum != ucRxBuffer[10]) {
			// 校验失败，丢弃本包
			ucRxCount = 0;
			return;
		}
		// 校验通过，解析数据
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;//加速度
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;//角速度
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;//角度
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
            default:break;
		}
		ucRxCount=0;	//清空缓存区
	}
}

/*********陀螺仪******/
void jy901s_G(){
    Gx=stcGyro.w[0]*2000/32768;
    Gy=stcGyro.w[1]*2000/32768;
    Gz=stcGyro.w[2]*2000/32768;
}
void jy901s_Angle(){
    Roll_x =stcAngle.Angle[0]*180/32768;
    Pitch_y=stcAngle.Angle[1]*180/32768;
    Yaw_z  =stcAngle.Angle[2]*180/32768;
}


//陀螺仪测试函数（没试过能不能用，如果不可以当我没有说（））
void jy901s_test(){
	float store_lastZ,store_nowZ;
	if(HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==0){
		store_nowZ=Yaw_z;
	}
	//(假设现在云台为0，0)
	if (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == 0)
	{
		stepper_move_step(x, angle_difference(store_lastZ, store_nowZ) * 1.8 / 16);
	}
}
//不用管的为^测试函数服务的函数们**********
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
    float angle_Kp = 6.88, angle_Kd = 3.42;
    // 假设向右转为正
    //  计算角度误差（考虑圆周特性）
    float angle_error = 0; // 当前误差
    angle_error = angle_difference(target_angle, current_angle);

    // PD控制器计算：(极性未验证)
    float output = angle_Kp * angle_error - angle_Kd * Gz;

    // 输出限幅
    if (output > 250)
    {
        output = 250;
    }
    else if (output < -250)
    {
        output = -250;
    }

    return (int)output;
}


