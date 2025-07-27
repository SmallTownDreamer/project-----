#include "table.h"
#include "main.h"
#include "oled.h"
// 按键1
int key1()
{
    return (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin));
}
// 按键2
int key2()
{
    return (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin));
}
// 按键3
int key3()
{
    return (HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin));
}

// 判断按键按下（阻塞式，待改）
int judge_key(int (*key_func)())
{
    if (key_func() == 1)
    {
        HAL_Delay(30);
        if (key_func() == 1)
        {
            while (key_func())
                ;
            return 1;
        }
    }
    return 0;
}

// 模式选择
short mode_choose()
{
    int key = 1;
    OLED_ShowString(0, 0, "choose mode", 16);

    while (1)
    {

        if (judge_key(key2) == 1)
            key--;
        if (judge_key(key3) == 1)
            key++;
        if (key > 3)
        {
            key = 1;
        }
        if (key < 1)
        {
            key = 3;
        }

        OLED_ShowString(32, 2, "MODE1     ", 16); // 第一页占空比
        OLED_ShowString(32, 4, "MODE2     ", 16); // 第二页脉冲数
        OLED_ShowString(32, 6, "MODE3     ", 16); // 第三页pid调控
        switch (key)
        {
        case 1:
            OLED_ShowNum(0, 2, 1, 1, 16);
            OLED_ShowChar(0, 4, ' ', 16);
            OLED_ShowChar(0, 6, ' ', 16);
            break;
        case 2:
            OLED_ShowChar(0, 2, ' ', 16);
            OLED_ShowChar(0, 6, ' ', 16);
            OLED_ShowNum(0, 4, 2, 1, 16);
            break;
        case 3:
            OLED_ShowChar(0, 2, ' ', 16);
            OLED_ShowChar(0, 4, ' ', 16);
            OLED_ShowNum(0, 6, 3, 1, 16);
            break;
        }

        if (judge_key(key1) == 1)
        {
            OLED_Clear();
            return key;
        }
    }
}

void mode1(){

}
void mode2(){

}
void mode3(){

}

