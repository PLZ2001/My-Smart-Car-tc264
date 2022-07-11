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
#include "SWITCH.h"
#include "LED.h"
#include "ICM.h"



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
            mt9v03x_finish_flag = 0;//��ʾ���Ը���mt9v03x_image��
            InsertTimer1Point(0);
            InsertTimer2Point(6);

            //Get_Cutted_Image();//�ü�ͼ��188*40


            InsertTimer1Point(1);


            if (Thresholding_Value_Init_Flag == 0)
            {

                Get_Inverse_Perspective_Table();//����͸�ӱ�
                //Get_Thresholding_Value();//���ֵ����ֵ
                GetBinThreshold_OSTU();//��򷨶�ֵ��
                time_up[3]=0.5f;
                Start_Timer(3);//������ʱ
                Thresholding_Value_Init_Flag = 1;
            }
            Get_Thresholding_Image();


            InsertTimer1Point(2);
            Get_Inverse_Perspective_Image();
            OLED_Camera_flag=1;

            InsertTimer1Point(3);

            //����Ĭ�ϴ�������λ��
            Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);

            //���������ҵı�ֱ�̶�
            Select_Left_Unknown_or_Right(6);

            //�����3�һ�����4����·�ڣ��Ҷ�ʱ��û���ڼ�ʱ���Ϳ���ʱ
            if (Read_Timer_Status(0) == PAUSED)
            {
                switch(classification_Result)
                {
                    case 2://�󻷵�
                        if (flag_For_Left_Circle == 0)//˵����û���󻷵�
                        {
                            flag_For_Left_Circle = 1;
                            //classification_Result = 7;//7����
//                            time_up[0] = rightCircle_RightTime;
//                            Start_Timer(0);
                        }
                        break;
                    case 3://�һ���
                        if (flag_For_Right_Circle == 0)//˵����û���һ���
                        {
                            flag_For_Right_Circle = 1;
                            //classification_Result = 8;//8����
//                            time_up[0] = rightCircle_RightTime;
//                            Start_Timer(0);
                        }
                        break;
                    case 4:
                        if (flag_For_ThreeRoad==0)
                        {
                            classification_Result = 4;//4����·��
                            flag_For_ThreeRoad = 1;
                        }

//                        time_up[0] = threeRoads_RightTime;
//                        Start_Timer(0);
                        break;
                    case 5:
                        classification_Result = 5;//5ʮ��·��
                        time_up[0] = 0.1f;
                        Start_Timer(0);
                        break;
                    case 10://��ֱ��
                        if (flag_For_Right_Circle == 2) //˵��׼�����һ���
                        {
                            flag_For_Right_Circle = 3;
                            //classification_Result = 7;//7����
//                            time_up[0] = rightCircle_LeftTime;
//                            Start_Timer(0);
                            time_up[4] = rightCircle_BannedTime;
                            Start_Timer(4);
                        }
                        break;
                    case 11://��ֱ��
                        if (flag_For_Left_Circle == 2) //˵��׼�����󻷵�
                        {
                            flag_For_Left_Circle = 3;
                            //classification_Result = 8;//8����
//                            time_up[0] = rightCircle_LeftTime;
//                            Start_Timer(0);
                            time_up[5] = rightCircle_BannedTime;
                            Start_Timer(5);
                        }
                        break;
                    case 14://T��
                        if (flag_For_T == 0)
                        {
                            if (flag_For_Right_T == 1||flag_For_Left_T == 1)
                            {
                                classification_Result = 14;
                                flag_For_T = 1;
                            }
                            else
                            {
                                flag_For_T = 0;
                            }
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
            if (Read_Timer_Status(5) == RUNNING)
            {
                if (Read_Timer(5)>time_up[5])
                {
                    Reset_Timer(5);
                }
            }
//            if (Read_Timer_Status(7) == RUNNING)
//            {
//                if (Read_Timer(7)>time_up[7])
//                {
//                    Reset_Timer(7);
//                }
//            }
            //������ڼ�ʱ����������
            if (Read_Timer_Status(0) == PAUSED)
            {
                if (flag_For_Right_Circle == 1)
                {
                    if (Left_Straight_Score<=2.7f)
                    {
                        flag_For_Right_Circle = 2;
                    }
                }
                else if (flag_For_Left_Circle == 1)
                {
                    if (Right_Straight_Score<=2.7f)
                    {
                        flag_For_Left_Circle = 2;
                    }
                }
                //С��������Բ��״̬
                else if (flag_For_Right_Circle == 2)
                {
//                    if (Check_Left_Straight(2,0,1) == 0)
                    if(Left_Straight_Score<=5.3f)
                    {
                        classification_Result = 3;//8;
                    }
                    else
                    {
                        classification_Result = 10;//10��ֱ��
                    }
                }
                //С��������Բ��״̬
                else if (flag_For_Left_Circle == 2)
                {
//                    if (Check_Right_Straight(0,-2,1) == 0)
                    if(Right_Straight_Score<=5.3f)
                    {
                        classification_Result = 2;//7;
                    }
                    else
                    {
                        classification_Result = 11;//11��ֱ��
                    }
                }
                else if (flag_For_T == 1)
                {
                    classification_Result = 14;
                    if (Check_TRoad(1,0.25f) == 1)
                    {
                        flag_For_T = 2;
//                        time_up[7] = T_Time;
//                        Start_Timer(7);
                    }
                }
                else if (flag_For_T == 2)
                {
//                    if (Read_Timer_Status(7) == PAUSED)
                    if (Left_Straight_Score>=3.0f||Unknown_Straight_Score>=3.0f||Right_Straight_Score>=3.0f)
                    {
                        flag_For_T=0;
                    }
                    else
                    {
                        if (flag_For_Right_T == 1)
                        {
                            classification_Result = 7;//����
                            flag_For_Right_T = 0;
                        }
                        if (flag_For_Left_T == 1)
                         {
                             classification_Result = 8;//����
                             flag_For_Left_T = 0;
                         }
                    }
                }
                else
                {//����ʶ��
                    if (flag_For_Right_Circle == 3 && Read_Timer_Status(4) == PAUSED)
                    {
                        flag_For_Right_Circle = 0;
                    }
                    if (flag_For_Left_Circle == 3 && Read_Timer_Status(5) == PAUSED)
                    {
                        flag_For_Left_Circle = 0;
                    }
                    if (flag_For_ThreeRoad == 1)
                    {
                        if(Left_Straight_Score<=2.7f)
                        {
                            flag_For_ThreeRoad = 0;
                        }
                    }
                    if (Check_Straight(1.0f))
                    {
                        classification_Result = 6;//6ֱ��
                    }
                    else
                    {
                        Classification_Classic36(0,&classification_Result,&classification_Result_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                        Check(&classification_Result,classification_Result_2nd);
                        Check(&classification_Result,9);
                    }
                    Check_Classification(classification_Result,1);

                    //�������´��ڵ�ʶ��
                    Set_Search_Range(0,height_Inverse_Perspective*6/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);
                    if (Check_Straight(0.5f))
                     {
                         classification_Result_1 = 6;//6ֱ��
                     }
                     else
                     {
                         Classification_Classic36(1,&classification_Result_1,&classification_Result_1_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                         Check(&classification_Result_1,classification_Result_1_2nd);
                         Check(&classification_Result_1,9);

                     }

                     //�������´��ڵ�ʶ��
                     Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);
                     if (Check_Straight(0.5f))
                      {
                          classification_Result_2 = 6;//6ֱ��
                      }
                      else
                      {
                          Classification_Classic36(2,&classification_Result_2,&classification_Result_2_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                          Check(&classification_Result_2,classification_Result_2_2nd);
                          Check(&classification_Result_2,9);
                      }


                     Set_Search_Range(0,height_Inverse_Perspective,width_Inverse_Perspective/4,width_Inverse_Perspective/2);
                     if (Check_Straight(0.6f))
                     {
                         classification_Result=6;
                         classification_Result_1=6;
                     }
                     //�Ļ�Ĭ�ϴ���
                     Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);

                     //��鳤ֱ���Ƿ�����
                     if((classification_Result_1==6||classification_Result_1==5) && (classification_Result==6||classification_Result==5))
                     {
                         Long_Straight_Flag = 1;//��ֱ��
                     }
                     else
                     {
                         Long_Straight_Flag = 0;//��ֱ��
                     }

                     if (classification_Result_2==2||classification_Result_2==3)
                     {
                         if (classification_Result == 7 || classification_Result == 8 || classification_Result == 9)
                         {
                             classification_Result = classification_Result_2;
                         }
                     }
                }
            }
            DrawCenterLine();
//            Compensate_ColCenter();


            //�ɴ�����ͼ�����Ϣ����ȡ�ٶȡ�ת��Ƕȵ�Ŀ��ֵ

            //Cal_Steering_Error(Get_d_steering_Error()<30.0f?0.5f:(Get_d_steering_Error()>120.0f?0.55f:((Get_d_steering_Error()-30.0f)/(120.0f-30.0f)*(0.55f-0.5f)+0.5f)));//����Col_Center��ɨ�跶Χsearch_Lines������ȫ�ֱ����������壩
            //if ((steering_Error>130||steering_Error<-130) && classification_Result == 9)
            //����������ʶ������Ϊ9���ߴ��ڻ����������仯�ʴ�ģ����ڻ�����ڽ׶ε�
//            if ((((d_steering_Error>0?d_steering_Error:-d_steering_Error)>30) && (classification_Result == 9 || (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1) ) )  )
//            {
//                Cal_Steering_Error(SightForward);
//                speed_Target = speed_Target_Low;
//
//                //Differential_Ratio = 2.5f;//1.3f;
//
//                Change_Steering_PID(0.25f,0,0.30f);
//                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//��Բ��˲�䵥����ת��pid
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//                else if (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//                else if (speed_Target_Low >= 2.2f && speed_Target_High >= 2.4f)//ֻ��2.1/1.9���ϲſ���
//                {
//                    Change_Steering_PID(0.27f,0,0.30f);
//                }
//            }
//            else
//            {
//                Cal_Steering_Error(SightForward);
//                speed_Target = speed_Target_High;
//
//                //Differential_Ratio = 2.5f;//1.3f
//                Change_Steering_PID(0.25f,0,0.30f);
//                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//��Բ��˲�䵥����ת��pid
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//                else if (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//
//
//            }



            //LEDָʾ
            if(Read_Timer_Status(0) == RUNNING)
            {
                LED_ON(1);
            }
            else
            {
                LED_OFF(1);
            }

            //LEDָʾ
            if(is_Slope==1)
            {
                LED_ON(2);
            }
            else
            {
                LED_OFF(2);
            }


            //ȷ���ٶ�״̬
            //"0����", "1����",
            //"2�󻷵�", "3�һ���",
            //"4����·��", "5ʮ��·��",
            //"6ֱ��","7������ʱʹ�ã�","8���ң���ʱʹ�ã�",
            //"9δ֪","10��ֱ��","11��ֱ��",
            //"12����","13�Ҷ���",
            //"14T��"

            uint8 set_flag=0;//��ʾ����Ƿ��������ٶ�


            if(Long_Straight_Flag == 1)
            {
                speed_Status = Highest;
                set_flag=1;
                time_up[6] = Highest_Distance/(speed_Target_Highest);
                Reset_Timer(6);
                Start_Timer(6);
            }
            else if (Read_Timer_Status(6) == RUNNING)
            {
                speed_Status = Highest;
                set_flag=1;
                if (Read_Timer(6)>time_up[6])
                {
                    Reset_Timer(6);
                }
            }

            if(classification_Result == 4)
            {
                set_flag=0;
                Reset_Timer(6);
            }


            static uint8 status_switch_1=0;
            if(steering_Target>37||steering_Target<-37)
            {
                status_switch_1=1;
            }
            if (steering_Target<25 && steering_Target>-25)
            {
                status_switch_1=0;
            }
            if (status_switch_1==1)
            {
                speed_Status = Low;
                set_flag=1;
                Reset_Timer(6);
            }


            static uint8 status_switch_2=0;
            if(d_steering_Error>50||d_steering_Error<-50)
            {
                status_switch_2=1;
            }
            if(steering_Target<10&&steering_Target>-10)
            {
                status_switch_2=0;
            }
            if (status_switch_2==1)
            {
                speed_Status = Lowest;
                set_flag=1;
                Reset_Timer(6);
            }

            if(is_Slope==1)
            {
                Reset_Timer(6);
                speed_Status = Lowest;
                set_flag=1;
            }

            if(classification_Result == 12||classification_Result == 13)
            {
                speed_Status = Lowest;
                Reset_Timer(6);
                set_flag=1;
                time_up[8] = 0.3f;
                Start_Timer(8);
            }
            else if (Read_Timer_Status(8) == RUNNING)
            {
                speed_Status = Lowest;
                set_flag=1;
                if (Read_Timer(8)>time_up[8])
                {
                    Reset_Timer(8);
                }
            }


            if(flag_For_T == 1)//T��
            {
                speed_Status = Lowest_ForT;
                Reset_Timer(6);
                set_flag=1;
            }
            else if(flag_For_T == 2)//T��
            {
                speed_Status = Lowest_ForT;
                Reset_Timer(6);
                set_flag=1;
            }


            if(set_flag==0)
            {
                speed_Status = High;
                Reset_Timer(6);
            }


            //�����ٶ�״̬������ؼ���

            float steeringPID_ratio_kp = 1.0f;
            float steeringPID_ratio_kd = 1.0f;
            float SightForward_ratio = 1.0f;
            float InnerSide_Ratio_ratio = 1.0f;
            if (classification_Result==2||classification_Result==3)
            {
                steeringPID_ratio_kp = 1.5f;//0.85f;
                steeringPID_ratio_kd = 0.1f;
                SightForward_ratio = 0.6f;
                InnerSide_Ratio_ratio = 1.0f;
            }

            if (is_Slope==1)
            {
                SightForward_ratio = 0.5f;
            }

            if (OLED_Camera_flag==1&&flag_for_ICM_Init==1)
            {
                static uint8 flag_Get_Volt_kd = 0;
                if (flag_Get_Volt_kd == 0)
                {
                    flag_Get_Volt_kd = 1;
                    Get_Volt_kd();
                    Cal_Differential_Ratio();
                }
                enum SpeedMode speed_Mode_temp = switch_Status[Switch1]+2*switch_Status[Switch2];
                if (speed_Mode_temp!=speed_Mode)
                {
                    speed_Mode = speed_Mode_temp;
                    Update_Speed_Mode();
                }
            }


            speed_Status = Filter_Speed_Status(speed_Status,12,18);
            if (classification_Result==2||classification_Result==3)
            {
                speed_Status = Low;
            }
            switch(speed_Status)
            {
                case Highest:
                {
                    SightForward = SightForward_Highest*SightForward_ratio;
                    speed_Target = speed_Target_Highest;
                    InnerSide_Ratio = InnerSide_Ratio_Highest*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_Highest[0]*steeringPID_ratio_kp,Steering_PID_Highest[1],Steering_PID_Highest[2]*steeringPID_ratio_kd);
                    break;
                }
                case High:
                {
                    SightForward = SightForward_High*SightForward_ratio;
                    speed_Target = speed_Target_High;
                    InnerSide_Ratio = InnerSide_Ratio_High*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_High[0]*steeringPID_ratio_kp,Steering_PID_High[1],Steering_PID_High[2]*steeringPID_ratio_kd);
                    break;
                }
                case Low:
                {
                    SightForward = SightForward_Low*SightForward_ratio;
                    speed_Target = speed_Target_Low;
                    InnerSide_Ratio = InnerSide_Ratio_Low*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_Low[0]*steeringPID_ratio_kp,Steering_PID_Low[1],Steering_PID_Low[2]*steeringPID_ratio_kd);
                    break;
                }
                case Lowest:
                {
                    SightForward = SightForward_Lowest*SightForward_ratio;
                    speed_Target = speed_Target_Lowest;
                    InnerSide_Ratio = InnerSide_Ratio_Lowest*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_Lowest[0]*steeringPID_ratio_kp,Steering_PID_Lowest[1],Steering_PID_Lowest[2]*steeringPID_ratio_kd);
                    break;
                }
                case Lowest_ForT:
                {
                    SightForward = SightForward_Lowest_ForT*SightForward_ratio;
                    speed_Target = speed_Target_Lowest_ForT;
                    InnerSide_Ratio = InnerSide_Ratio_Lowest_ForT*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_Lowest_ForT[0]*steeringPID_ratio_kp,Steering_PID_Lowest_ForT[1],Steering_PID_Lowest_ForT[2]*steeringPID_ratio_kd);
                    break;
                }
            }
            Cal_Steering_Error(SightForward);
            Cal_Steering_Target();//����ȫ�ֱ����������壩����λ��ʽPDԭ����ת��Ŀ��Steering_Target(��Χ-30~30��������ת��������ת)

            if(Check_TRoad(0,0.1))
            {
                emergency_Stop=1;
            }

            UART_Flag_TX = TRUE;

            InsertTimer1Point(4);
        }



        //����Ŀ���ҵ���ʱ������
        switch(speed_Mode)
        {
            case Lowest_Mode:
            {
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
                break;
            }
            case Low_Mode:
            {
                if (speed_Target1 < 0.5 && speed_Target1 > -0.5 && speed_Measured1 < 0.5 && speed_Measured1 > -0.5)
                {
                    PID_mode1 = OPEN_LOOP1;
                }
                else
                {
                    if (speed_Measured1 > BANGBANG_UP + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN + speed_Target1)
                    {
                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
                    }
                    else
                    {
                        PID_mode1 = PID_CLOSED_LOOP1;
                    }
                }

                if (speed_Target2 < 0.5 && speed_Target2 > -0.5 && speed_Measured2 < 0.5 && speed_Measured2 > -0.5)
                {
                    PID_mode2 = OPEN_LOOP2;
                }
                else
                {
                    if (speed_Measured2 > BANGBANG_UP + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN + speed_Target2)
                    {
                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
                    }
                    else
                    {
                        PID_mode2 = PID_CLOSED_LOOP2;
                    }
                }
                break;
            }
            case High_Mode:
            {
                if (speed_Target1 < 0.5 && speed_Target1 > -0.5 && speed_Measured1 < 0.5 && speed_Measured1 > -0.5)
                {
                    PID_mode1 = OPEN_LOOP1;
                }
                else
                {
                    if (speed_Measured1 > BANGBANG_UP + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN + speed_Target1)
                    {
                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
                    }
                    else
                    {
                        PID_mode1 = PID_CLOSED_LOOP1;
                    }
                }

                if (speed_Target2 < 0.5 && speed_Target2 > -0.5 && speed_Measured2 < 0.5 && speed_Measured2 > -0.5)
                {
                    PID_mode2 = OPEN_LOOP2;
                }
                else
                {
                    if (speed_Measured2 > BANGBANG_UP + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN + speed_Target2)
                    {
                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
                    }
                    else
                    {
                        PID_mode2 = PID_CLOSED_LOOP2;
                    }
                }
                break;
            }
            case Highest_Mode:
            {
                if (speed_Target1 < 0.5 && speed_Target1 > -0.5 && speed_Measured1 < 0.5 && speed_Measured1 > -0.5)
                {
                    PID_mode1 = OPEN_LOOP1;
                }
//                else if (speed_Measured1 < 1.0 && speed_Measured1 > -1.0)
//                {
//                    PID_mode1 = PID_CLOSED_LOOP1;
//                }
                else
                {
                    if (speed_Measured1 > BANGBANG_UP + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN + speed_Target1)
                    {
                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
                    }
                    else
                    {
                        PID_mode1 = PID_CLOSED_LOOP1;
                    }
                }

                if (speed_Target2 < 0.5 && speed_Target2 > -0.5 && speed_Measured2 < 0.5 && speed_Measured2 > -0.5)
                {
                    PID_mode2 = OPEN_LOOP2;
                }
//                else if (speed_Measured2 < 1.0 && speed_Measured2 > -1.0)
//                {
//                    PID_mode2 = PID_CLOSED_LOOP2;
//                }
                else
                {
                    if (speed_Measured2 > BANGBANG_UP + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN + speed_Target2)
                    {
                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
                    }
                    else
                    {
                        PID_mode2 = PID_CLOSED_LOOP2;
                    }
                }
                break;
            }
        }



    }
}



#pragma section all restore
