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
#include "KEY.h"//����ɨ�����
#include "CAMERA.h"//����ͷ��ͼ�������
#include "MOTOR.h"//ֱ��������
#include "OLED.h"//��ʾ�����
#include "STEERING.h"//������
#include "UART.h"
#include "fastlz.h"
#include "TIME.h"


#pragma section all "cpu1_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��



void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //�û��ڴ˴����ø��ֳ�ʼ��������





	//�ȴ����к��ĳ�ʼ�����
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //�����������ͼ�񵽴�󣬽���ͼ����
        if (mt9v03x_finish_flag == 1 && (UART_Flag_TX == FALSE || UART_EN == FALSE))
        {
            Get_Cutted_Image();//�ü�ͼ��188*40
            mt9v03x_finish_flag = 0;//��ʾ���Ը���mt9v03x_image��

            Get_Thresholding_Image();
            Get_Inverse_Perspective_Image();

            static uint8 flag_For_Right_Circle = 0;
            //�����3�һ������ģ��Ҷ�ʱ��û���ڼ�ʱ���Ϳ���ʱ
            if (Read_Timer_Status() == PAUSED && classification_Result == 3)
            {
                if (flag_For_Right_Circle == 0)//˵����û���һ���
                {
                    flag_For_Right_Circle = 1;
                    classification_Result = 14;//����
                }
                else //˵��׼�����һ���
                {
                    flag_For_Right_Circle = 0;
                    classification_Result = 13;//����
                }
                Start_Timer();
            }
            //����ڼ�ʱ���жϼ�ʱ�Ƿ�ﵽҪ��ʱ��
            if (Read_Timer_Status() == RUNNING)
            {
                switch (classification_Result)
                {
                    case 13:
                        if (Read_Timer()>1.0f) //13�����Ǽ�ʱ1s
                        {
                            Reset_Timer();
                        }
                        break;
                    case 14:
                        if (Read_Timer()>3.0f) //14�����Ǽ�ʱ3s
                        {
                            Reset_Timer();
                        }
                        break;
                    default:
                        break;
                }
            }
            //������ڼ�ʱ����������
            if (Read_Timer_Status() == PAUSED)
            {
                //С��������Բ��״̬
                if (flag_For_Right_Circle == 1)
                {
                    if (Check_Straight())
                    {
                        classification_Result = 12;
                    }
                    else
                    {
                        classification_Result = Classification();
                    }
                    Check_Classification(classification_Result,10);
                    //ֻ�е��ٴ�ʶ����Բ������ʱ���ſ���flag_For_Right_Circle=0���Ӷ�����������ʶ�𣬷���һֱ������ʻ
                    if (classification_Result != 3)
                    {
                        classification_Result = 14;
                    }
                }
                else
                {//����ʶ��
                    if (Check_Straight())
                    {
                        classification_Result = 12;
                    }
                    else
                    {
                        classification_Result = Classification();
                    }
                    Check_Classification(classification_Result,10);
                }

            }

            DrawCenterLine();

            //�ɴ�����ͼ�����Ϣ����ȡ�ٶȡ�ת��Ƕȵ�Ŀ��ֵ
            Cal_Steering_Error(0.5);//����Col_Center��ɨ�跶Χsearch_Lines������ȫ�ֱ����������壩
            Cal_Speed_Target();//����Col_Center��ɨ�跶Χsearch_Lines�����ٶ�Ŀ��speed_Target�������

            if (UART_EN == TRUE)
            {
                UART_Flag_TX = TRUE;
            }
        }



        //����Ŀ���ҵ���ʱ������
        if (speed_Target < 0.5 && speed_Target > -0.5 && speed_Measured < 0.5 && speed_Measured > -0.5)
        {
            PID_mode = OPEN_LOOP;
        }
        else
        {
            PID_mode = PID_CLOSED_LOOP;
        }



    }
}



#pragma section all restore
