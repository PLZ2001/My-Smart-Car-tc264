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


uint8 start_Flag = 0;//1表示启动差速函数
uint8 emergency_Stop = 0;//1表示紧急停车

float Differential_Ratio = 1.0f;
float InnerSide_Ratio = 1.05f;//1.0f;//0.85f;
float InnerSide_Ratio_Highest = 1.05f;
float InnerSide_Ratio_High = 1.05f;
float InnerSide_Ratio_Low = 1.05f;
float InnerSide_Ratio_Lowest = 1.05f;

float Steering_PID_Highest[3]={0.25f,0,0.30f};
float Steering_PID_High[3]={0.25f,0,0.30f};
float Steering_PID_Low[3]={0.25f,0,0.30f};
float Steering_PID_Lowest[3]={0.25f,0,0.30f};


float Base_Volt = 8.30f;//基准电压
float Real_Volt = 8.30f;//真实电压

enum SpeedStatus speed_Status = High;
enum SpeedMode speed_Mode = 10;//不等于0、1、2、3就行

void Differential_Motor(void)
{
    if (steering_Error>=0)
    {
        //我艹有bug
        speed_Target2 = speed_Target + speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f;//左轮目标速度（m/s）
        speed_Target1 = speed_Target - speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//左轮目标速度（m/s）
//        speed_Target1 = speed_Target - 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f;//右轮目标速度（m/s）
        speed_Target2 = speed_Target + speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//右轮目标速度（m/s）
//        speed_Target2 = speed_Target + 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);;
    }
}


void Update_Speed_Mode(void)
{
    switch (speed_Mode)
    {
        case Lowest_Mode:
        {
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

            break;
        }
        case Low_Mode:
        {
            speed_Target_Highest = 1.5*2.5f;
            SightForward_Highest = 0.25f;
            InnerSide_Ratio_Highest = 1.15f;
            Steering_PID_Highest[0]=0.18f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.20f;


            speed_Target_High = 2.5f;//即2.2
            SightForward_High = 0.25f;
            InnerSide_Ratio_High = 1.10f;
            Steering_PID_High[0]=0.18f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.20f;


            speed_Target_Low = 2.1f;//即1.9
            SightForward_Low = 0.25f;
            InnerSide_Ratio_Low = 1.05f;
            Steering_PID_Low[0]=0.22f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.20f;


            speed_Target_Lowest = 0.7*2.1f;
            SightForward_Lowest = 0.25f;
            InnerSide_Ratio_Lowest = 1.00f;
            Steering_PID_Lowest[0]=0.30f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.20f;

            break;
        }
        case High_Mode:
        {
            speed_Target_Highest = 2.0*2.8f;
            SightForward_Highest = 0.25f;
            InnerSide_Ratio_Highest = 1.50f;
            Steering_PID_Highest[0]=0.25f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.30f;


            speed_Target_High = 2.8f;//即2.5
            SightForward_High = 0.25f;
            InnerSide_Ratio_High = 1.20f;
            Steering_PID_High[0]=0.25f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.30f;


            speed_Target_Low = 2.4f;//即2.1
            SightForward_Low = 0.25f;
            InnerSide_Ratio_Low = 1.00f;
            Steering_PID_Low[0]=0.25f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.30f;


            speed_Target_Lowest = 0.7*2.4f;
            SightForward_Lowest = 0.25f;
            InnerSide_Ratio_Lowest = 1.00f;
            Steering_PID_Lowest[0]=0.30f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.30f;

            break;
        }
        case Highest_Mode:
        {
            speed_Target_Highest = 2.0*3.1f;
            SightForward_Highest = 0.25f;
            InnerSide_Ratio_Highest = 1.50f;
            Steering_PID_Highest[0]=0.25f;Steering_PID_Highest[1]=0;Steering_PID_Highest[2]=0.30f;


            speed_Target_High = 3.1f;//即2.7
            SightForward_High = 0.25f;
            InnerSide_Ratio_High = 1.20f;
            Steering_PID_High[0]=0.25f;Steering_PID_High[1]=0;Steering_PID_High[2]=0.30f;


            speed_Target_Low = 2.7f;//即2.4
            SightForward_Low = 0.25f;
            InnerSide_Ratio_Low = 1.00f;
            Steering_PID_Low[0]=0.25f;Steering_PID_Low[1]=0;Steering_PID_Low[2]=0.30f;


            speed_Target_Lowest = 0.7*2.7f;
            SightForward_Lowest = 0.25f;
            InnerSide_Ratio_Lowest = 1.00f;
            Steering_PID_Lowest[0]=0.30f;Steering_PID_Lowest[1]=0;Steering_PID_Lowest[2]=0.30f;

            break;
        }
    }
}
