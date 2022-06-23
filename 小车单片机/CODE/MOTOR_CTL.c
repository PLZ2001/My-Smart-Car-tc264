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
float speed_Target_Min = 2.8;//"2.8m/s"ʵ����2.5m/s
float speed_Target_Max = 2.8;

uint8 start_Flag = 0;//1��ʾ�������ٺ���
uint8 emergency_Stop = 0;//1��ʾ����ͣ��

float Differential_Ratio = 1.0f;
float InnerSide_Ratio = 1.05f;//1.0f;//0.85f;

float Base_Volt = 8.30f;//��׼��ѹ
float Real_Volt = 8.30f;//��ʵ��ѹ

void Differential_Motor(void)
{
    if (steering_Error>=0)
    {
        //��ܳ��bug
        speed_Target2 = speed_Target + speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f;//����Ŀ���ٶȣ�m/s��
        speed_Target1 = speed_Target - speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target2 = speed_Target + steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//����Ŀ���ٶȣ�m/s��
//        speed_Target1 = speed_Target - 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);
    }
    else
    {
        speed_Target1 = speed_Target - speed_Target*Differential_Ratio*steering_Target*4.0f/1062.5f;//����Ŀ���ٶȣ�m/s��
        speed_Target2 = speed_Target + speed_Target*InnerSide_Ratio*Differential_Ratio*steering_Target*4.0f/1062.5f;
//        speed_Target1 = speed_Target - steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);//����Ŀ���ٶȣ�m/s��
//        speed_Target2 = speed_Target + 0.9*steering_Error/600*(speed_Target+Differential_Ratio)/(1.2f+1);;
    }
}



