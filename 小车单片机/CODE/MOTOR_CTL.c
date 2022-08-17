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
float OuterSide_Ratio = 1.0f;
float OuterSide_Ratio_Highest = 1.0f;
float OuterSide_Ratio_High = 1.0f;
float OuterSide_Ratio_Low = 1.0f;
float OuterSide_Ratio_Lowest = 1.0f;
float OuterSide_Ratio_Lowest_ForT = 1.0f;
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




float Highest_Distance = 0.6f;

float Volt_kd=0;
float Volt_DR=0;

float Min_DR = 1.85f;//1.75f;
float Max_DR = 1.85;//1.75f;

void Differential_Motor(void)
{
    float last_steering_Target=0;
    if (steering_Error>=0)
    {
        speed_Target2 = speed_Target + /*0.15f**/0.65f*speed_Target*OuterSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f+0*speed_Target*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;//左轮目标速度（m/s）
        speed_Target1 = speed_Target - 0.75f*speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f - 0*speed_Target*InnerSide_Ratio*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;
//        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//左轮目标速度（m/s）
//        speed_Target1 = speed_Target - 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - speed_Target*OuterSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f- 0*speed_Target*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;//右轮目标速度（m/s）
        speed_Target2 = speed_Target + 3.0f*speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f + 0*speed_Target*InnerSide_Ratio*Differential_Ratio*(steering_Target-last_steering_Target)*4.0f/1062.5f;
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

            T_Time = 0.5f;

            T_Line = 0.09f;

            threeRoads_RightTime = 0.25f;
            ThreeeRoad_Delay = 0.07f;

            Highest_Distance = 0.7f;

            speed_Target_Highest =1.5*2.1f;
            SightForward_Highest = 0.25f;
            OuterSide_Ratio_Highest = 1.0f;
            InnerSide_Ratio_Highest = 1.50f;
            Steering_PID_Highest[0]=0.18f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.30f;


            speed_Target_High = 2.1f;//即1.9
            SightForward_High = 0.25f;
            OuterSide_Ratio_High = 1.0f;
            InnerSide_Ratio_High = 1.20f;
            Steering_PID_High[0]=0.18f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.30f;


            speed_Target_Low = 1.7f;//即1.5
            SightForward_Low = 0.25f;
            InnerSide_Ratio_Low = 1.00f;
            OuterSide_Ratio_Low = 1.0f;
            Steering_PID_Low[0]=0.22f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.30f;


            speed_Target_Lowest = 0.7*1.7f;
            SightForward_Lowest = 0.25f;
            InnerSide_Ratio_Lowest = 1.00f;
            OuterSide_Ratio_Lowest = 1.0f;
            Steering_PID_Lowest[0]=0.30f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.30f;

            speed_Target_Lowest_ForT = 0.7*1.7f;
            SightForward_Lowest_ForT = 0.25f;
            InnerSide_Ratio_Lowest_ForT = 1.00f;
            OuterSide_Ratio_Lowest_ForT = 1.0f;
            Steering_PID_Lowest_ForT[0]=0.30f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.30f;

            break;
        }
        case Low_Mode:
        {

            Min_DR = 1.85f;//1.75f;
            Max_DR = 1.85;//1.75f;
            Differential_Ratio = Volt_DR;

            T_Time = 0.12f;

            T_Line = 0.09f;

            threeRoads_RightTime = 0.6f;//0.25f;
            ThreeeRoad_Delay = 0;//0.07f+0.02f;

            Highest_Distance = 0.05f;

            BANGBANG_UP1 = 1.0;
            BANGBANG_DOWN1 = 1.0;
            BANGBANG_UP2 = 1.0;
            BANGBANG_DOWN2 = 1.0;

            speed_Target_Highest = 3.3f;//即2.9
            SightForward_Highest = 0.35f;
            OuterSide_Ratio_Highest = 0.9f;
            InnerSide_Ratio_Highest = 1.1f;
            Steering_PID_Highest[0]=0.14f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=1.0f;


            speed_Target_High = 3.2f;//即2.8
            SightForward_High = 0.35f;
            OuterSide_Ratio_High = 0.9f;
            InnerSide_Ratio_High = 1.1f;//1.15f;
            Steering_PID_High[0]=0.14f;Steering_PID_High[1]=0;Steering_PID_High[2]=1.0f;


            speed_Target_Low = 3.1f;//即2.7
            SightForward_Low = 0.35f;
            OuterSide_Ratio_Low = 0.9f;
            InnerSide_Ratio_Low = 1.1f;//1.25;
            Steering_PID_Low[0]=0.15f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=1.0f;


            speed_Target_Lowest = 3.0f;//即2.6
            SightForward_Lowest = 0.35f;
            OuterSide_Ratio_Lowest = 0.9f;
            InnerSide_Ratio_Lowest = 1.1f;
            Steering_PID_Lowest[0]=0.15f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=1.0f;

            speed_Target_Lowest_ForT = 2.4f;//即2.1
            SightForward_Lowest_ForT = 0.40f;
            OuterSide_Ratio_Lowest_ForT = 1.0f;
            InnerSide_Ratio_Lowest_ForT = 3.00f;
            //Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
            Steering_PID_Lowest_ForT[0]=0.13f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.8f;

            break;
        }
        case High_Mode:
        {

//            Differential_Ratio = Volt_DR;
//
//            T_Time = 0.12f;
//
//            T_Line = 0.18f;
//
//            threeRoads_RightTime = 0.15f;
//            ThreeeRoad_Delay = 0.04f;
//
//            Highest_Distance = 0.05f;
//
//            BANGBANG_UP1 = 1.0;
//            BANGBANG_DOWN1 = 1.0;
//            BANGBANG_UP2 = 1.0;
//            BANGBANG_DOWN2 = 1.0;
//
//            speed_Target_Highest = 3.9f;//即3.4
//            SightForward_Highest = 0.38f;
//            OuterSide_Ratio_Highest = 1.1f;
//            InnerSide_Ratio_Highest = 1.15f;
//            Steering_PID_Highest[0]=0.14f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=1.0f;
//
//
//            speed_Target_High = 3.5f;//即3.1
//            SightForward_High = 0.38f;
//            OuterSide_Ratio_High = 1.1f;
//            InnerSide_Ratio_High = 1.15f;//1.15f;
//            Steering_PID_High[0]=0.14f;Steering_PID_High[1]=0;Steering_PID_High[2]=1.0f;
//
//
//            speed_Target_Low = 3.3f;//即2.9
//            SightForward_Low = 0.38f;
//            OuterSide_Ratio_Low = 1.2f;
//            InnerSide_Ratio_Low = 1.25f;//1.25;
//            Steering_PID_Low[0]=0.155f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=1.0f;
//
//
//            speed_Target_Lowest = 3.25f;//即2.85
//            SightForward_Lowest = 0.38f;
//            OuterSide_Ratio_Lowest = 1.2f;
//            InnerSide_Ratio_Lowest = 1.25f;
//            Steering_PID_Lowest[0]=0.155f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=1.0f;
//
//            speed_Target_Lowest_ForT = 2.4f;//即2.1
//            SightForward_Lowest_ForT = 0.40f;
//            OuterSide_Ratio_Lowest_ForT = 1.0f;
//            InnerSide_Ratio_Lowest_ForT = 3.00f;
//            //Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
//            Steering_PID_Lowest_ForT[0]=0.13f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.8f;
//
//            break;
            Min_DR = 1.75f;//1.75f;
            Max_DR = 1.75f;//1.75f;
            Differential_Ratio = Volt_DR;

            T_Time = 0.12f;

            T_Line = 0.09f+0.09f;

            threeRoads_RightTime = 0.6f;//0.25f;
            ThreeeRoad_Delay = 0.00f;

            Highest_Distance = 0.05f;

            BANGBANG_UP1 = 1.0;
            BANGBANG_DOWN1 = 1.0;
            BANGBANG_UP2 = 1.0;
            BANGBANG_DOWN2 = 1.0;

            speed_Target_Highest = 7.0f;//即6.2
            SightForward_Highest = 0.43f;
            OuterSide_Ratio_Highest = 0.3f;
            InnerSide_Ratio_Highest = 0.3f;
            Steering_PID_Highest[0]=0.06f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.4f;


            speed_Target_High = 3.7f+0.5f;//即3.3
            SightForward_High = 0.43f;
            OuterSide_Ratio_High = 0.9f;
            InnerSide_Ratio_High = 1.15f;//1.15f;
            Steering_PID_High[0]=0.14f;Steering_PID_High[1]=0;Steering_PID_High[2]=1.1f;


            speed_Target_Low = 3.6f+0.5f;//即3.2
            SightForward_Low = 0.43f;
            OuterSide_Ratio_Low = 0.9f;
            InnerSide_Ratio_Low = 1.15f;//1.25;
            Steering_PID_Low[0]=0.14f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=1.1f;


            speed_Target_Lowest = 3.5f+0.5f;//即3.1
            SightForward_Lowest = 0.43f;
            OuterSide_Ratio_Lowest = 0.9f;
            InnerSide_Ratio_Lowest = 1.15f;
            Steering_PID_Lowest[0]=0.14f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=1.1f;

            speed_Target_Lowest_ForT = 2.4f;//即2.1
            SightForward_Lowest_ForT = 0.40f;
            OuterSide_Ratio_Lowest_ForT = 2.5f;
            InnerSide_Ratio_Lowest_ForT = 5.00f;
            //Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
            Steering_PID_Lowest_ForT[0]=0.13f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.8f;

            break;
        }
        case Highest_Mode:
        {
            Differential_Ratio = Volt_DR;

            T_Time = 0.12f;

            T_Line = 0.18f;

            threeRoads_RightTime = 0.15f;
            ThreeeRoad_Delay = 0.00f;

            Highest_Distance = 0.05f;

            BANGBANG_UP1 = 1.0;
            BANGBANG_DOWN1 = 1.0;
            BANGBANG_UP2 = 1.0;
            BANGBANG_DOWN2 = 1.0;

            speed_Target_Highest = 4.0f;//即3.5
            SightForward_Highest = 0.43f;
            OuterSide_Ratio_Highest = 1.15f;
            InnerSide_Ratio_Highest = 1.35f;
            Steering_PID_Highest[0]=0.12f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.8f;


            speed_Target_High = 3.6f;//即3.2
            SightForward_High = 0.43f;
            OuterSide_Ratio_High = 1.15f;
            InnerSide_Ratio_High = 1.35f;//1.15f;
            Steering_PID_High[0]=0.12f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.8f;


            speed_Target_Low = 3.5f;//即3.1
            SightForward_Low = 0.43f;
            OuterSide_Ratio_Low = 1.15f;
            InnerSide_Ratio_Low = 1.35f;//1.25;
            Steering_PID_Low[0]=0.13f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.8f;


            speed_Target_Lowest = 3.45f;//即3.05
            SightForward_Lowest = 0.43f;
            OuterSide_Ratio_Lowest = 1.15f;
            InnerSide_Ratio_Lowest = 1.35f;
            Steering_PID_Lowest[0]=0.13f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.8f;

            speed_Target_Lowest_ForT = 2.4f;//即2.1
            SightForward_Lowest_ForT = 0.40f;
            OuterSide_Ratio_Lowest_ForT = 1.0f;
            InnerSide_Ratio_Lowest_ForT = 3.00f;
            //Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
            Steering_PID_Lowest_ForT[0]=0.13f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.8f;

            break;
//            Differential_Ratio = Volt_DR;
//
//            T_Time = 0.12f;
//
//            Highest_Distance = 0.05f;
//
//            BANGBANG_UP1 = 1.0;
//            BANGBANG_DOWN1 = 1.0;
//            BANGBANG_UP2 = 1.0;
//            BANGBANG_DOWN2 = 1.0;
//
//            speed_Target_Highest = 1.5*3.9f;
//            SightForward_Highest = 0.44f;
//            OuterSide_Ratio_Highest = 1.0f;
//            InnerSide_Ratio_Highest = 1.15f;
//            Steering_PID_Highest[0]=0.10f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.6f;
//
//
//            speed_Target_High = 3.9f;//即3.4
//            SightForward_High = 0.44f;
//            OuterSide_Ratio_High = 1.0f;
//            InnerSide_Ratio_High = 1.20f;//1.15f;
//            Steering_PID_High[0]=0.11f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.6f;
//
//
//            speed_Target_Low = 3.4f;//即3.0
//            SightForward_Low = 0.44f;
//            OuterSide_Ratio_Low = 1.0f;
//            InnerSide_Ratio_Low = 1.15f;//1.25;
//            Steering_PID_Low[0]=0.11f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.6f;
//
//
//            speed_Target_Lowest = 3.1f;//即2.7
//            SightForward_Lowest = 0.44f;
//            OuterSide_Ratio_Lowest = 1.0f;
//            InnerSide_Ratio_Lowest = 1.10f;
//            Steering_PID_Lowest[0]=0.10f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.6f;
//
//            speed_Target_Lowest_ForT = 2.4f;//即2.1
//            SightForward_Lowest_ForT = 0.44f;
//            OuterSide_Ratio_Lowest_ForT = 1.0f;
//            InnerSide_Ratio_Lowest_ForT = 3.00f;
//            //Steering_PID_Lowest_ForT[0]=2.50f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.10f;
//            Steering_PID_Lowest_ForT[0]=0.10f;Steering_PID_Lowest_ForT[1]=0;Steering_PID_Lowest_ForT[2]=0.6f;
//
//            break;
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
        Volt_DR= Max_DR;//1.86f;
    }
    else if (Real_Volt<8.15f)
    {
        Volt_DR= Min_DR;//1.85f;
    }
    else
    {
        Volt_DR= (Min_DR+(Max_DR-Min_DR)/(8.30f-8.15f)*(Real_Volt-8.15f));
    }
}
