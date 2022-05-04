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

uint8 start_Flag = 0;//1表示启动差速函数
uint8 emergency_Stop = 0;//1表示紧急停车


void Differential_Motor(void)
{
    if (steering_Error>=0)
    {
        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+1.1f)/(1.2f+1);//左轮目标速度（m/s）
        speed_Target1 = speed_Target;
    }
    else
    {
        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+1.1f)/(1.2f+1);//右轮目标速度（m/s）
        speed_Target2 = speed_Target;
    }
}
