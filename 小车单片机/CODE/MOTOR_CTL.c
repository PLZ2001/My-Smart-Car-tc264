#include "headfile.h"
#include "IfxGpt12_reg.h"
#include "IfxGpt12.h"
#include "fuzzy_PID.h"//ģ��PID�㷨
#include "MOTOR_CTL.h"
#include "MOTOR1.h"
#include "MOTOR2.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"

// ��ѵ�ص�ѹ��8.2V


float speed_Target;//Ŀ���ٶ�
float speed_Target_Min = 1.8;//"1.8m/s"ʵ����1.6m/s
float speed_Target_Max;

uint8 start_Flag = 0;//1��ʾ�������ٺ���
uint8 emergency_Stop = 0;//1��ʾ����ͣ��

float Differential_Ratio = 1.1f;


void Differential_Motor(void)
{
    if (steering_Error>=0)
    {
        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//����Ŀ���ٶȣ�m/s��
        speed_Target1 = speed_Target - 0.7*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//����Ŀ���ٶȣ�m/s��
        speed_Target2 = speed_Target + 0.7*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);;
    }
}

