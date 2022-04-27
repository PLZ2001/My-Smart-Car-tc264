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
#include "OLED.h"//��ʾ�����
#include "STEERING.h"//������
#include "UART.h"
#include "fastlz.h"
#include "TIME.h"
#include "SEARCH.h"
#include "MOTOR1.h"//ֱ��������
#include "MOTOR2.h"//ֱ��������
#include "MOTOR_CTL.h"



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
            InsertTimer1Point(0);
            InsertTimer2Point(6);

            Get_Cutted_Image();//�ü�ͼ��188*40
            mt9v03x_finish_flag = 0;//��ʾ���Ը���mt9v03x_image��

            InsertTimer1Point(1);

            //Get_ICM_DATA();//��������������

            if (Thresholding_Value_Init_Flag == 0)
            {
                Get_Inverse_Perspective_Table();//����͸�ӱ�
                Get_Thresholding_Value();//���ֵ����ֵ
                time_up[3]=1.0f;
                Start_Timer(3);//������ʱ
                Thresholding_Value_Init_Flag = 1;
            }
            Get_Thresholding_Image();

            InsertTimer1Point(2);
            Get_Inverse_Perspective_Image();

            InsertTimer1Point(3);


            static uint8 flag_For_Right_Circle = 0;
            //�����3�һ�����4����·�ڣ��Ҷ�ʱ��û���ڼ�ʱ���Ϳ���ʱ
            if (Read_Timer_Status(0) == PAUSED)
            {
                switch(classification_Result)
                {
                    case 3:
                        if (flag_For_Right_Circle == 0)//˵����û���һ���
                        {
                            flag_For_Right_Circle = 1;
                            classification_Result = 8;//8����
                            time_up[0] = rightCircle_RightTime;
                            Start_Timer(0);
                        }
                        break;
                    case 4:
                        classification_Result = 4;//4����·��
                        time_up[0] = threeRoads_RightTime;
                        Start_Timer(0);
                        break;
                    case 10:
                        if (flag_For_Right_Circle == 1) //˵��׼�����һ���
                        {
                            flag_For_Right_Circle = 2;
                            classification_Result = 7;//7����
                            time_up[0] = rightCircle_LeftTime;
                            Start_Timer(0);
                            time_up[4] = rightCircle_BannedTime;
                            Start_Timer(4);
                        }
                        break;
                    default:
                        break;
                }

            }
            //����ڼ�ʱ���жϼ�ʱ�Ƿ�ﵽҪ��ʱ��
            if (Read_Timer_Status(0) == RUNNING)
            {
                if (Read_Timer(0)>time_up[0])
                {
                    Reset_Timer(0);
                }
            }
            if (Read_Timer_Status(4) == RUNNING)
            {
                if (Read_Timer(4)>time_up[4])
                {
                    Reset_Timer(4);
                }
            }
            //������ڼ�ʱ����������
            if (Read_Timer_Status(0) == PAUSED)
            {
                //С��������Բ��״̬
                if (flag_For_Right_Circle == 1)
                {
                    //ֻ�е��ٴ�ʶ����Բ��ʱ���ſ���flag_For_Right_Circle=0���Ӷ�����������ʶ�𣬷���һֱ8������ʻ
                    if (Check_Left_Straight(2,0) == 0)
                    {
                        classification_Result = 8;
                    }
                    else
                    {
                        classification_Result = 10;//10��ֱ��
                    }
                }
                else
                {//����ʶ��
                    if (flag_For_Right_Circle == 2 && Read_Timer_Status(4) == PAUSED)
                    {
                        flag_For_Right_Circle = 0;
                    }
                    if (Check_Straight())
                    {
                        classification_Result = 6;//6ֱ��
                    }
                    else
                    {
                        classification_Result = Classification_25();//������㷨Classification_25()����ͳ�����㷨Classification_Classic()
                        if (classification_Result ==3)//3�һ���
                        {
                            if(flag_For_Right_Circle!=0 || !Check_RightCircle())
                            {
                                classification_Result = 9;//9δ֪
                            }
                        }
                        if (classification_Result ==4)//4����·��
                        {
                            if(!Check_ThreeRoads())
                            {
                                classification_Result = 9;//9δ֪
                            }
                        }
                        if (classification_Result == 9)//9δ֪
                        {
                            if(Check_Left_Straight(2,-2))
                            {
                                classification_Result = 7;//7����
                            }
                            if(Check_Right_Straight(2,-2))
                            {
                                classification_Result = 8;//8����
                            }
                        }

                    }
                    Check_Classification(classification_Result,1);
                }

            }

            DrawCenterLine();

            //�ɴ�����ͼ�����Ϣ����ȡ�ٶȡ�ת��Ƕȵ�Ŀ��ֵ
            Cal_Steering_Error(0.5);//����Col_Center��ɨ�跶Χsearch_Lines������ȫ�ֱ����������壩




            if (UART_EN == TRUE)
            {
                UART_Flag_TX = TRUE;
            }

            InsertTimer1Point(4);
        }



        //����Ŀ���ҵ���ʱ������
        if (speed_Target1 < 0.5 && speed_Target1 > -0.5 && speed_Measured1 < 0.5 && speed_Measured1 > -0.5)
        {
            PID_mode1 = OPEN_LOOP1;
        }
        else
        {
            PID_mode1 = PID_CLOSED_LOOP1;
        }

        if (speed_Target2 < 0.5 && speed_Target2 > -0.5 && speed_Measured2 < 0.5 && speed_Measured2 > -0.5)
        {
            PID_mode2 = OPEN_LOOP2;
        }
        else
        {
            PID_mode2 = PID_CLOSED_LOOP2;
        }



    }
}



#pragma section all restore
