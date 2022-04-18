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


float speed_Target = 0;//目标速度

uint8 start_Flag = 0;  //1表示启动


void Differential_Motor(void)
{
    speed_Target1 = speed_Target - steering_Error/600*(speed_Target+1)/(1.2f+1);//右轮目标速度（m/s）
    speed_Target2 = speed_Target + steering_Error/600*(speed_Target+1)/(1.2f+1);//左轮目标速度（m/s）
}
