#include "control.h"

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
void LASER_ON(){
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
}
void LASER_OFF(){
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
}
    