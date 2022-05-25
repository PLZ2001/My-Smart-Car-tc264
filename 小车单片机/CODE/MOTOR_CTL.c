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
float speed_Target_Min = 1.8;//"1.8m/s"实际是1.6m/s
float speed_Target_Max = 1.8;

uint8 start_Flag = 0;//1表示启动差速函数
uint8 emergency_Stop = 0;//1表示紧急停车

float Differential_Ratio = 3.4f;
float InnerSide_Ratio = 0.85f;


void Differential_Motor(void)
{
    if (steering_Error>=0)
    {
        speed_Target2 = speed_Target + Differential_Ratio*steering_Target*4.0f/1062.5f;//左轮目标速度（m/s）
        speed_Target1 = speed_Target - InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//左轮目标速度（m/s）
//        speed_Target1 = speed_Target - 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - Differential_Ratio*steering_Target*4.0f/1062.5f;//右轮目标速度（m/s）
        speed_Target2 = speed_Target + InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//右轮目标速度（m/s）
//        speed_Target2 = speed_Target + 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);;
    }
}



