/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#include "CAMERA.h"//����ͷ��ͼ�������
#include "SWITCH.h"
#include "KEY.h"//����ɨ�����
#include "OLED.h"//��ʾ�����
#include "STEERING.h"//������
#include "UART.h"//����ͨ�����
#include "fastlz.h"//ѹ���㷨
#include "fuzzy_PID.h"//ģ��PID�㷨
#include "SEARCH.h"
#include "WIFI.h"
#include "MOTOR1.h"//ֱ��������
#include "MOTOR2.h"//ֱ��������
#include "ICM.h"
#include "ADC.h"
#include "LED.h"
//#include "EEPROM.h"


#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��
int core0_main(void)
{
    disableInterrupts();
	get_clk();//��ȡʱ��Ƶ��  ��ر���
	//�û��ڴ˴����ø��ֳ�ʼ��������
//    EEPROM_Write_Data(ID_STEERING_DUTY_CENTER, &STEERING_DUTY_CENTER);//��д��ȥ
//	STEERING_DUTY_CENTER = EEPROM_Read_Data(ID_STEERING_DUTY_CENTER,uint32);//�ٶ�ȡ


	My_Init_Steering();//�ҵĳ�ʼ�����
	My_Init_Motor1();//�ҵĳ�ʼ��ֱ�����
	My_Init_Motor2();//�ҵĳ�ʼ��ֱ�����
	My_Init_SpeedSensor1();//�ҵĳ�ʼ��������
	My_Init_SpeedSensor2();//�ҵĳ�ʼ��������
    My_Init_OLED();//�ҵĳ�ʼ��OLED
    My_Init_Camera();//�ҵĳ�ʼ������ͷ
    My_Init_Switch();//�ҵĳ�ʼ������
    My_Init_Key();//�ҵĳ�ʼ������
    My_Init_UART();//�ҵĳ�ʼ������ͨ��
    My_Init_FuzzyPID_Speed();//�ҵĳ�ʼ���ٶ�ģ��PID����
    My_Init_Timer();//�ҵĳ�ʼ��TIMER

    My_Init_ADC();//�ҵĳ�ʼ��ADC



    My_Init_LED();
    //My_Init_Wifi();//�ҵĳ�ʼ��WIFIͨ��

    //�ȴ����к��ĳ�ʼ�����
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();

    //ע�� ��V1.1.6�汾֮��  printf��ӡ����Ϣ�Ӵ�������������ѧϰ������6-Printf_Demo
	while (TRUE)
	{
        if (flag_for_ICM_Init == 0)
        {
//            VL53L0X_Init();
            My_Init_ICM();//�ҵĳ�ʼ��ICM
            Get_Zero_Bias();//����������Ưֵ
            flag_for_ICM_Init = 1;
        }

        Get_ADC_DATA();//���µ�ѹ��ȡ

//        Lazer_Data = VL53L0X_GetValue();

        Get_ICM_DATA();//��������������
        Check_Slope_with_YHF();

	    if (UART_EN == TRUE)
	    {
	        UART(Send);
	    }
	    else
	    {
	        UART(Emergency_Send);
	    }
	    //Send_with_Wifi();//��wifi����

	}
}



#pragma section all restore


