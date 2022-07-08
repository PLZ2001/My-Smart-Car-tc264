#include "headfile.h"
#include "IfxGpt12_reg.h"
#include "IfxGpt12.h"
#include "fuzzy_PID.h"//模糊PID算法
#include "MOTOR_CTL.h"
#include "MOTOR1.h"
#include "MOTOR2.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"

// 最佳电池电压：8.2V



float speed_Target;//目标速度
float speed_Target_Low = 1.7;//2.8;//"2.8m/s"实际是2.5m/s
float speed_Target_High = 2.1;//2.8;
float speed_Target_Highest = 2.0*2.8;
float speed_Target_Lowest = 0.7*2.8;
float speed_Target_Lowest_ForT = 0.7*2.8;


uint8 start_Flag = 0;//1表示启动差速函数
uint8 emergency_Stop = 0;//1表示紧急停车

float Differential_Ratio = 1.0f;
float InnerSide_Ratio = 1.05f;//1.0f;//0.85f;
float InnerSide_Ratio_Highest = 1.05f;
float InnerSide_Ratio_High = 1.05f;
float InnerSide_Ratio_Low = 1.05f;
float InnerSide_Ratio_Lowest = 1.05f;
float InnerSide_Ratio_Lowest_ForT = 1.05f;

float Steering_PID_Highest[3]={0.25f,0,0.30f};
float Steering_PID_High[3]={0.25f,0,0.30f};
float Steering_PID_Low[3]={0.25f,0,0.30f};
float Steering_PID_Lowest[3]={0.25f,0,0.30f};
float Steering_PID_Lowest_ForT[3]={0.25f,0,0.30f};


float Base_Volt = 8.30f;//基准电压
float Real_Volt = 8.30f;//真实电压

enum SpeedStatus speed_Status = High;
enum SpeedMode speed_Mode = 10;//不等于0、1、2、3就行


float BANGBANG_UP=0.2;
float BANGBANG_DOWN=0.3;

float Highest_Distance = 0.6f;

float Volt_kd=0;
float Volt_DR=0;

void Differential_Motor(void)
{
    float last_steering_Target=0;
    if (steering_Error>=0)
    {
        speed_Target2 = speed_Target + speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f+0*speed_Target*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;//左轮目标速度（m/s）
        speed_Target1 = speed_Target - speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f - 0*speed_Target*InnerSide_Ratio*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;
//        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//左轮目标速度（m/s）
//        speed_Target1 = speed_Target - 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f- 0*speed_Target*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;//右轮目标速度（m/s）
        speed_Target2 = speed_Target + speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f + 0*speed_Target*InnerSide_Ratio*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;
//        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//右轮目标速度（m/s）
//        speed_Target2 = speed_Target + 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);;
    }
    last_steering_Target = steering_Target;
}

void UART_Speed_Mode(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x15);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    int16 speed_target = (int16)round(1000*speed_Target);
    int16 speed_target1 = (int16)round(1000*speed_Target1);
    int16 speed_target2 = (int16)round(1000*speed_Target2);
    int16 speed_measured1 = (int16)round(1000*speed_Measured1);
    int16 speed_measured2 = (int16)round(1000*speed_Measured2);
    int16 steering_target = (int16)round(100*steering_Target);
    int16 steering_error = (int16)round(10*steering_Error);
    int16 d_steering_error = (int16)round(100*d_steering_Error);
    int16 speed_status = (int16)round(speed_Status);
    int16 class = (int16)round(classification_Result);
    uart_putchar(DEBUG_UART, speed_target>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_target&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_target1>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_target1&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_target2>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_target2&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_measured1>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_measured1&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_measured2>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_measured2&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, steering_target>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, steering_target&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, steering_error>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, steering_error&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, d_steering_error>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, d_steering_error&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_status>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, speed_status&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, class>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, class&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x15);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}
void Update_Speed_Mode(void)
{
    switch (speed_Mode)
    {
        case Lowest_Mode:
        {
            Differential_Ratio = 1.0f;

            T_Time = 1.0f;

            Highest_Distance = 0.7f;

            speed_Target_Highest =1.5*2.1f;
            SightForward_Highest = 0.25f;
            InnerSide_Ratio_Highest = 1.50f;
            Steering_PID_Highest[0]=0.18f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.30f;


            speed_Target_High = 2.1f;//即1.9
            SightForward_High = 0.25f;
            InnerSide_Ratio_High = 1.20f;
            Steering_PID_High[0]=0.18f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.30f;


            speed_Target_Low = 1.7f;//即1.5
            SightForward_Low = 0.25f;
            InnerSide_Ratio_Low = 1.00f;
            Steering_PID_Low[0]=0.22f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.30f;


            speed_Target_Lowest = 0.7*1.7f;
            SightForward_Lowest = 0.25f;
            InnerSide_Ratio_Lowest = 1.00f;
            Steering_PID_Lowest[0]=0.30f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.30f;

            speed_Target_Lowest_ForT = 0.7*1.7f;
            SightForward_Lowest_ForT = 0.25f;
            InnerSide_Ratio_Lowest_ForT = 1.00f;
            Steering_PID_Lowest_ForT[0]=0.30f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.30f;

            break;
        }
        case Low_Mode:
        {
            Differential_Ratio = 1.0f;

            T_Time = 1.0f;

            Highest_Distance = 0.5f;

            BANGBANG_UP = 0.3;//0.2;
            BANGBANG_DOWN = 0.5;//0.3;

            speed_Target_Highest = 1.5*2.5f;
            SightForward_Highest = 0.28f;
            InnerSide_Ratio_Highest = 1.05f;
            Steering_PID_Highest[0]=0.18f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.20f;


            speed_Target_High = 2.5f;//即2.2
            SightForward_High = 0.28f;
            InnerSide_Ratio_High = 1.10f;
            Steering_PID_High[0]=0.19f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.20f;


            speed_Target_Low = 2.1f;//即1.9
            SightForward_Low = 0.28f;
            InnerSide_Ratio_Low = 1.05f;
            Steering_PID_Low[0]=0.23f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.20f;


            speed_Target_Lowest = 0.7*2.1f;
            SightForward_Lowest = 0.28f;
            InnerSide_Ratio_Lowest = 1.20f;
            Steering_PID_Lowest[0]=0.50f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.20f;

            speed_Target_Lowest_ForT = 0.7*2.1f;
            SightForward_Lowest_ForT = 0.28f;
            InnerSide_Ratio_Lowest_ForT = 1.20f;
            Steering_PID_Lowest_ForT[0]=0.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.20f;

            break;
        }
        case High_Mode:
        {
//            Differential_Ratio = 0.85f;
//
//            T_Time = 0.4f;
//
//            Highest_Distance = 0.35f;
//
//            BANGBANG_UP = 0.2;//0.2;
//            BANGBANG_DOWN = 0.4;//0.3;
//
//            speed_Target_Highest = 1.5*2.8f;
//            SightForward_Highest = 0.30f;
//            InnerSide_Ratio_Highest = 1.00f;
//            Steering_PID_Highest[0]=0.17f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.30f;
//
//
//            speed_Target_High = 2.8f;//即2.5
//            SightForward_High = 0.30f;
//            InnerSide_Ratio_High = 1.00f;
//            Steering_PID_High[0]=0.17f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.30f;
//
//
//            speed_Target_Low = 2.5f;//即2.2
//            SightForward_Low = 0.30f;
//            InnerSide_Ratio_Low = 1.10f;
//            Steering_PID_Low[0]=0.21f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.30f;
//
//
//            speed_Target_Lowest = 0.7*2.5f;
//            SightForward_Lowest = 0.30f;
//            InnerSide_Ratio_Lowest = 1.60f;
//            Steering_PID_Lowest[0]=0.80f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.10f;
//
//            speed_Target_Lowest_ForT = 0.7*2.5f;
//            SightForward_Lowest_ForT = 0.30f;
//            InnerSide_Ratio_Lowest_ForT = 1.60f;
//            Steering_PID_Lowest_ForT[0]=0.80f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
            Differential_Ratio = 0.95f;

            T_Time = 0.4f;

            Highest_Distance = 0.2f;

            BANGBANG_UP = 0.2;
            BANGBANG_DOWN = 0.2;

            speed_Target_Highest = 1.5*3.6f;
            SightForward_Highest = 0.30f;
            InnerSide_Ratio_Highest = 1.15f;
            Steering_PID_Highest[0]=0.12f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=Volt_kd;


            speed_Target_High = 3.6f;//即3.2
            SightForward_High = 0.30f;
            InnerSide_Ratio_High = 1.15f;//1.15f;
            Steering_PID_High[0]=0.12f;Steering_PID_High[1]=0;Steering_PID_High[2]=Volt_kd;


            speed_Target_Low = 3.1f;//即2.7
            SightForward_Low = 0.30f;
            InnerSide_Ratio_Low = 1.15f;//1.25;
            Steering_PID_Low[0]=0.12f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=Volt_kd;


            speed_Target_Lowest = 2.65f;//即2.34
            SightForward_Lowest = 0.30f;
            InnerSide_Ratio_Lowest = 1.15f;
            Steering_PID_Lowest[0]=0.12f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=Volt_kd;

            speed_Target_Lowest_ForT = 2.4f;//即2.1
            SightForward_Lowest_ForT = 0.40f;
            InnerSide_Ratio_Lowest_ForT = 1.70f;
            Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;

            break;
        }
        case Highest_Mode:
        {
            Differential_Ratio = Volt_DR;

            T_Time = 0.8f;

            Highest_Distance = 0.05f;

            BANGBANG_UP = 1.0;
            BANGBANG_DOWN = 1.0;

            speed_Target_Highest = 1.5*3.9f;
            SightForward_Highest = 0.44f;
            InnerSide_Ratio_Highest = 1.15f;
            Steering_PID_Highest[0]=0.10f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.6f;


            speed_Target_High = 3.9f;//即3.4
            SightForward_High = 0.44f;
            InnerSide_Ratio_High = 1.10f;//1.15f;
            Steering_PID_High[0]=0.10f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.6f;


            speed_Target_Low = 3.4f;//即3.0
            SightForward_Low = 0.44f;
            InnerSide_Ratio_Low = 1.1f;//1.25;
            Steering_PID_Low[0]=0.11f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.6f;


            speed_Target_Lowest = 3.1f;//即2.7
            SightForward_Lowest = 0.44f;
            InnerSide_Ratio_Lowest = 1.10f;
            Steering_PID_Lowest[0]=0.10f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.6f;

            speed_Target_Lowest_ForT = 2.4f;//即2.1
            SightForward_Lowest_ForT = 0.44f;
            InnerSide_Ratio_Lowest_ForT = 2.50f;
            Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;

            break;
        }
    }
}


int Filter_Speed_Status(int status,int cnt_limit,int cnt_limit_toHigh)
{
    static int last_status;//上一次的
    static int filter_status;//稳定的结果
    static int cnt_now=0;
    static int flag = 0;//用来初始化
    if (flag == 0)
    {
        last_status = status;
        filter_status = status;
        flag=1;
    }
    else
    {
        if (status<last_status||status == 4)//4即Highest
        {
            cnt_now=0;
            last_status = status;
            filter_status = status;
        }
        else if (status == last_status)
        {
            cnt_now++;
            if (status == 3)//3即High
            {
                if (cnt_now>=cnt_limit_toHigh)
                {
                    filter_status = status;
                }
            }
            else
            {
                if (cnt_now>=cnt_limit)
                {
                    filter_status = status;
                }
            }

        }
        else
        {
            cnt_now=0;
            last_status = status;
        }
    }
    return filter_status;
}

void Get_Volt_kd(void)
{
    if (Real_Volt>8.30f)
    {
        Volt_kd = 1.20f;
    }
    else if (Real_Volt<8.20f)
    {
        Volt_kd = 1.10f;
    }
    else
    {
        Volt_kd = 1.10f+(1.20f-1.10f)/(8.30f-8.20f)*(Real_Volt-8.20f);
    }
}

float Cal_Differential_Ratio(void)
{
    if (Real_Volt>8.30f)
    {
        Volt_DR= 1.35f;
    }
    else if (Real_Volt<8.15f)
    {
        Volt_DR= 1.80f;
    }
    else
    {
        Volt_DR= (1.80f+(1.35f-1.80f)/(8.30f-8.15f)*(Real_Volt-8.15f));
    }
}
