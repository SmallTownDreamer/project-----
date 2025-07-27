#include "track.h"
#include "main.h"
#include <math.h>
// //送药小车
// void track(int speed){
    
// }

//也是用的取离散点(假设是256*256的分辨率（不重要权重的问题罢了）)
//计算量可以再优化比如权重映射到函数上，就可以实现直接计算得到误差，不用取离散点了
static uint8_t weight_y[5] = {60, 100, 140, 180, 220};  // y坐标采样点
static uint8_t weight_factor[5] = {1.0, 0.8, 0.5, 0.3, 0.1};  // 对应权重


float kp,ki,kd;
int a,b;//y=ax+b
#define CENTER_X 256/2
int error_sum,error_last;
void pid_test(){
    //省略读取(得直线方程为y=ax+b)

    uint8_t now_err = 0;

    //对取样点进行加权误差累计计算
    for(int i = 0; i < 5; i++) {
        float y = weight_y[i];
        float weight = weight_factor[i];
        float x=(y-b)/a;
        
        // 计算偏差 (正向中心为x=0，即图像中心)
        float point_error = x - CENTER_X;
        
        // 加权累加
        now_err += point_error * weight;
    } 
    
    // 积分项 
    error_sum += now_err;
    if(error_sum > 1000) error_sum = 1000;
    if(error_sum < -1000) error_sum = -1000;
    
    // 微分项
    float err_d=now_err - error_last;
    error_last = now_err;
    
    // PID输出
    float pid_output = kp* now_err+ki* error_sum + kp*err_d;



}
