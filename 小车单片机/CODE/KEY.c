#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//显示屏相关
#include "UART.h"
#include "MOTOR_CTL.h"
#include "SEARCH.h"
#include "CAMERA.h"
#include "STEERING.h"
#include "MOTOR1.h"
#include "MOTOR2.h"
//#include "EEPROM.h"



void My_Init_Key(void)
{
    gpio_init(KEY0_GPIO, GPI, 0, NO_PULL);//KEY0输入
    gpio_init(KEY1_GPIO, GPI, 0, NO_PULL);//KEY1输入
    gpio_init(KEY2_GPIO, GPI, 0, NO_PULL);//KEY2输入
    gpio_init(KEY3_GPIO, GPI, 0, NO_PULL);//KEY2输入
}

//按键初始状态为0
//如果状态0或状态1的某按键低电平，计数加一
//如果状态0或状态1的某按键一旦高电平，则回到状态0，计数清零
//如果处于状态0的某按键计数达到FIRST_COUNTER_MAX，则进入状态1，并执行相应动作，计数清零
//如果处于状态1的某按键计数达到SECOND_COUNTER_MAX，则留在状态1，并执行相应动作，计数清零
//该函数要求10ms执行一次
void Check_Key_per10ms(void)
{
    static uint8 counter[4], status[4], trigger[4];
    //检查Key0
    if (gpio_get(KEY0_GPIO)==0)
    {
        counter[Key0]++;
        if (counter[Key0]>=(status[Key0]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key0] = 1;
            trigger[Key0] = TRUE;
            counter[Key0] = 0;
        }
    }
    else
    {
        status[Key0] = 0;
        counter[Key0] = 0;
    }
    //检查Key1
    if (gpio_get(KEY1_GPIO)==0)
    {
        counter[Key1]++;
        if (counter[Key1]>=(status[Key1]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key1] = 1;
            trigger[Key1] = TRUE;
            counter[Key1] = 0;
        }
    }
    else
    {
        status[Key1] = 0;
        counter[Key1] = 0;
    }
    //检查Key2
    if (gpio_get(KEY2_GPIO)==0)
    {
        counter[Key2]++;
        if (counter[Key2]>=(status[Key2]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key2] = 1;
            trigger[Key2] = TRUE;
            counter[Key2] = 0;
        }
    }
    else
    {
        status[Key2] = 0;
        counter[Key2] = 0;
    }
    //检查Key3
    if (gpio_get(KEY3_GPIO)==0)
    {
        counter[Key3]++;
        if (counter[Key3]>=(status[Key3]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key3] = 1;
            trigger[Key3] = TRUE;
            counter[Key3] = 0;
        }
    }
    else
    {
        status[Key3] = 0;
        counter[Key3] = 0;
    }
    //执行动作
    if (trigger[Key0])
    {
        Key0_Action();
        trigger[Key0] = FALSE;
    }
    if (trigger[Key1])
    {
        Key1_Action();
        trigger[Key1] = FALSE;
    }
    if (trigger[Key2])
    {
        Key2_Action();
        trigger[Key2] = FALSE;
    }
    if (trigger[Key3])
    {
        Key3_Action();
        trigger[Key3] = FALSE;
    }
}

void Key0_Action(void)
{
    if (OLED_EN)
    {
        OLED_Page++;
        if (OLED_Page >= PAGE_NUM)
        {
            OLED_Page = 0;
        }
    }
    OLED_Page_Active_Flag = TRUE;
}

void Key1_Action(void)
{
    switch (OLED_Page)
    {
        case UART_Setting_Page:
            UART_EN = UART_EN?FALSE:TRUE;
            break;
        case OLED_Setting_Page:
            OLED_EN = OLED_EN?FALSE:TRUE;
            break;
        case Differential_Page:
            pointer_temp += 1;
            if (pointer_temp >= 3)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case Volt_Page:
            pointer_temp += 1;
            if (pointer_temp >= 2)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case Steering_Center_Page:
            pointer_temp += 1;
            if (pointer_temp >= 1)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case SteeringPID_Page:
            pointer_temp += 1;
            if (pointer_temp >= 3)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case TimeSet_Page:
            pointer_temp += 1;
            if (pointer_temp >= 5)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case MotorPID_Page:
            pointer_temp += 1;
            if (pointer_temp >= 3)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case Speed_Page:
            start_Flag = start_Flag?0:1;
            break;
        default:
            break;
    }
    OLED_Page_Active_Flag = TRUE;
}

void Key2_Action(void)
{
    switch (OLED_Page)
    {
        case Camera_Page:{
            static int8 direction = 1;
            fuzzy_thresholdingValue_36+=0.05*direction;
            if (fuzzy_thresholdingValue_36>=1)
            {
                direction = -1;
            }
            if (fuzzy_thresholdingValue_36<=0)
            {
                direction = 1;
            }
            break;}
        case UART_Setting_Page:
            UART_EN = UART_EN?FALSE:TRUE;
            break;
        case OLED_Setting_Page:
            OLED_EN = OLED_EN?FALSE:TRUE;
            break;
        case SteeringPID_Page:
            Key_temp();
            break;
        case Differential_Page:
            switch (pointer_temp)
            {
                case 0:
                    Differential_Ratio += 0.05f*up_Down;
                    break;
                case 1:
                    InnerSide_Ratio += 0.05f*up_Down;
                    break;
                case 2:
                    SightForward +=0.01f*up_Down;
                default:
                    break;
            }
            break;
        case Volt_Page:
            switch (pointer_temp)
            {
                case 0:
                    Base_Volt += 0.01f*up_Down;
                    break;
                case 1:
                    Real_Volt += 0.01f*up_Down;
                    break;
                default:
                    break;
            }
            break;
        case Steering_Center_Page:
            switch (pointer_temp)
            {
                case 0:
                    STEERING_DUTY_CENTER += 2*up_Down;
                    break;
                default:
                    break;
            }
//            EEPROM_Write_Data(ID_STEERING_DUTY_CENTER, &STEERING_DUTY_CENTER);
            break;
        case TimeSet_Page:
            switch (pointer_temp)
            {
                case 0:
                    threeRoads_RightTime += 0.01f*up_Down;
                    break;
                case 1:
                    rightCircle_RightTime += 0.1f*up_Down;
                    break;
                case 2:
                    rightCircle_LeftTime += 0.1f*up_Down;
                    break;
                case 3:
                    rightCircle_BannedTime += 0.5f*up_Down;
                    break;
                case 4:
                    T_Time += 0.1f*up_Down;
                default:
                    break;
            }
            break;
        case MotorPID_Page:
            switch (pointer_temp)
            {
                case 0:
                    PID_KP1 += 0.05f*up_Down;
                    PID_KP2 += 0.05f*up_Down;
                    break;
                case 1:
                    PID_KI1 += 0.05f*up_Down;
                    PID_KI2 += 0.05f*up_Down;
                    break;
                case 2:
                    PID_KD1 += 0.05f*up_Down;
                    PID_KD2 += 0.05f*up_Down;
                    break;
                default:
                    break;
            }
            break;
        case Speed_Page:{
            static int8 direction = 1;
            if (speed_Target_High>=2.6)
            {
                speed_Target_High+=0.1*direction;
            }
            else
            {
                speed_Target_High+=0.2*direction;
            }

            if (speed_Target_High>=4)
            {
                direction = -1;
            }
            if (speed_Target_High<=-4)
            {
                direction = 1;
            }
            break;}
        default:
            break;
    }
    OLED_Page_Active_Flag = TRUE;
}

void Key3_Action(void)
{
    switch (OLED_Page)
    {
        case Speed_Page:{
            static int8 direction = 1;
            if (speed_Target_Low>=2.6)
            {
                speed_Target_Low+=0.1*direction;
            }
            else
            {
                speed_Target_Low+=0.2*direction;
            }

            if (speed_Target_Low>=4)
            {
                direction = -1;
            }
            if (speed_Target_Low<=-4)
            {
                direction = 1;
            }
            break;}
        default:
            break;
    }
    OLED_Page_Active_Flag = TRUE;
}
