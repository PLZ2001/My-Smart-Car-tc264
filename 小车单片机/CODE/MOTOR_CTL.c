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
float speed_Target_Min = 2.8;//"2.8m/s"实际是2.5m/s
float speed_Target_Max = 2.8;

uint8 start_Flag = 0;//1表示启动差速函数
uint8 emergency_Stop = 0;//1表示紧急停车

float Differential_Ratio = 1.0f;
float InnerSide_Ratio = 1.05f;//1.0f;//0.85f;

float Base_Volt = 8.30f;//基准电压
float Real_Volt = 8.30f;//真实电压

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



