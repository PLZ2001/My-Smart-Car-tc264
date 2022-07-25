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
#include "rtthread.h"
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
#include "MOTOR_CTL.h"
#include "TIMETIME.h"
#include "RT_HELPER.h"

#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// ================������ֲ��ʾ================
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ================������ֲ��ʾ================


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��
//��������


void Image_Process(void)
{
    Get_Thresholding_Image();
    Get_Inverse_Perspective_Image();
    OLED_Camera_flag=1;
}

void Update_Left_Unknown_or_Right(void)
{
    //���������ҵı�ֱ�̶�
    Select_Left_Unknown_or_Right(9);
}

void Check_Special_Classification(void)
{
    //�����3�һ�����4����·�ڣ��Ҷ�ʱ��0û���ڼ�ʱ���Ϳ���ʱ
     if (Read_Timer_Status(0) == PAUSED)
     {
         switch(classification_Result)
         {
             case 2://�󻷵�
                 if (flag_For_Left_Circle == 0)//˵����û���󻷵�
                 {
                     flag_For_Left_Circle = 1;
                     time_up[0] = rightCircle_RightTime;
                     Start_Timer(0);
                 }
                 break;
             case 3://�һ���
                 if (flag_For_Right_Circle == 0)//˵����û���һ���
                 {
                     flag_For_Right_Circle = 1;
                     time_up[0] = rightCircle_RightTime;
                     Start_Timer(0);
                 }
                 break;
             case 4:
                 time_up[0] = threeRoads_RightTime;
                 Start_Timer(0);
                 break;
             case 5:
                 classification_Result = 5;//5ʮ��·��
                 break;
             case 10://��ֱ��
                 if (flag_For_Right_Circle == 2) //˵��׼�����һ���
                 {
                     flag_For_Right_Circle = 3;
                     time_up[4] = rightCircle_BannedTime;
                     Start_Timer(4);
                 }
                 break;
             case 11://��ֱ��
                 if (flag_For_Left_Circle == 2) //˵��׼�����󻷵�
                 {
                     flag_For_Left_Circle = 3;
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
         if (Read_Timer(0)<ThreeeRoad_Delay)
         {
             ThreeeRoad_Delay_Flag = 0;
         }
         else
         {
             ThreeeRoad_Delay_Flag = 1;
         }
     }
}

void Basic_Classification(void)
{
    //����Ĭ�ϴ�������λ��
    Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
    Helper_Window_Flag = 0;
    //������ڼ�ʱ����������
     if (Read_Timer_Status(0) == PAUSED)
     {
         if (flag_For_Right_Circle == 1)
         {
             if (Left_Straight_Score<=2.7f)
             {
                 flag_For_Right_Circle = 2;
                 time_up[13] = 0.75f;
                 Start_Timer(13);
             }
         }
         else if (flag_For_Left_Circle == 1)
         {
             if (Right_Straight_Score<=2.7f)
             {
                 flag_For_Left_Circle = 2;
                 time_up[13] = 0.75f;
                 Start_Timer(13);
             }
         }
         //С��������Բ��״̬
         else if (flag_For_Right_Circle == 2)
         {
             if (Check_Left_Straight(2,0,1) == 0 || Read_Timer_Status(13) == RUNNING)
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
             if (Check_Right_Straight(0,-2,1) == 0 || Read_Timer_Status(13) == RUNNING)
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
             if (Check_TRoad(1,T_Line,3) == 1)
             {
                 flag_For_T = 2;
                 time_up[7] = 0.2f;
                 Start_Timer(7);
                 time_up[14] = 0.3f;
                 Start_Timer(14);
             }
         }
         else if (flag_For_T == 2)
         {
             if (((Left_Straight_Score>=2.6f||Unknown_Straight_Score>=2.6f||Right_Straight_Score>=2.6f)  && Read_Timer(7)==PAUSED) || Read_Timer(14)==PAUSED)
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
             if (Check_Straight(1.0f))
             {
                 classification_Result = 6;//6ֱ��
             }
             else if (is_Slope == 1 || is_Slope == 2)
             {
                 classification_Result = 9;
                 Check(&classification_Result,9);
             }
             else
             {
                 Classification_Classic36(0,&classification_Result,&classification_Result_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                 Check(&classification_Result,classification_Result_2nd);
                 Check(&classification_Result,9);
             }

             //�������´��ڵ�ʶ��
             Set_Search_Range(0,height_Inverse_Perspective*6/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
             if (Check_Straight(0.5f))
             {
                 classification_Result_1 = 6;//6ֱ��
             }
             else if (is_Slope != 0)
             {
                 classification_Result = 9;
                 Check(&classification_Result,9);
             }
             else
             {
                 Classification_Classic36(1,&classification_Result_1,&classification_Result_1_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                 Check(&classification_Result_1,classification_Result_1_2nd);
                 Check(&classification_Result_1,9);

             }

             //�����Ǹ������ڵ�ʶ��
             Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
             if (Check_Straight(0.5f))
             {
                 classification_Result_2 = 6;//6ֱ��
             }
             else if (is_Slope != 0)
             {
                 classification_Result = 9;
                 Check(&classification_Result,9);
             }
             else
             {
                 Classification_Classic36(2,&classification_Result_2,&classification_Result_2_2nd);//������㷨Classification_25()����ͳ�����㷨Classification_Classic()��ģ����·��Classification_Classic36()
                 Check(&classification_Result_2,classification_Result_2_2nd);
                 Check(&classification_Result_2,9);
             }


             Set_Search_Range(0,height_Inverse_Perspective,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
             if (Check_Straight(0.7f))
             {
                 classification_Result=6;
                 classification_Result_1=6;
             }
             //�Ļ�Ĭ�ϴ���
             Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);

             //��鳤ֱ���Ƿ�����
             if((classification_Result_1==6||classification_Result_1==5) && (classification_Result==6))
             {
                 Long_Straight_Flag = 1;//��ֱ��
             }
             else
             {
                 Long_Straight_Flag = 0;//��ֱ��
             }

             // �������ڵ�����
             if (classification_Result_2==2||classification_Result_2==3||classification_Result_2==5||classification_Result_2==4||classification_Result_2==12||classification_Result_2==13)
             {
                 if (classification_Result == 7 || classification_Result == 8 || classification_Result == 9)
                 {
                     classification_Result = classification_Result_2;
                     Helper_Window_Flag = 1;
                 }
                 else
                 {
                     Helper_Window_Flag = 0;
                 }
             }
         }
     }
}

void Confirm_Speed(void)
{
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

    if(is_Slope==1 || is_Slope == 2 || is_Slope == 3)
    {
        Reset_Timer(6);
        speed_Status = Lowest;
        set_flag=1;
    }

    if(classification_Result == 12||classification_Result == 13 ||classification_Result == 5)
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

    if (zebra_status == starting)
    {
        speed_Status = Lowest_ForT;
        Reset_Timer(6);
        set_flag=1;
    }
    else if (zebra_status == finishing)
    {
        speed_Status = Lowest_ForZebra;
        Reset_Timer(6);
        set_flag=1;
    }


    if(set_flag==0)
    {
        speed_Status = High;
        Reset_Timer(6);
    }
}

void Confirm_Motion(void)
{
    float steeringPID_ratio_kp = 1.0f;
    float steeringPID_ratio_kd = 1.0f;
    float SightForward_ratio = 1.0f;
    float OuterSide_Ratio_ratio = 1.0f;
    float InnerSide_Ratio_ratio = 1.0f;
    float speed_Target_ratio = 1.0f;

    if(flag_For_T == 2)//T��
    {
        speed_Target_ratio = 0.9f;
    }

    if (flag_For_Left_Circle == 2)
    {
        speed_Target_ratio = 0.95f;
        steeringPID_ratio_kp = 2.3f;//0.85f;
        steeringPID_ratio_kd = 0.6f;
        SightForward_ratio = 0.6f;
        OuterSide_Ratio_ratio = 1.5f;
        InnerSide_Ratio_ratio = 1.1f;
    }

    if (flag_For_Left_Circle == 1)
    {
        speed_Target_ratio = 0.95f;
        steeringPID_ratio_kp = 2.3f;//0.85f;
        steeringPID_ratio_kd = 0.6f;
        SightForward_ratio = 0.6f;
        OuterSide_Ratio_ratio = 1.5f;
        InnerSide_Ratio_ratio = 1.1f;
    }


    //��Բ����ֵ�ƫС

    if (flag_For_Right_Circle == 2)
    {
        speed_Target_ratio = 0.95f;
        steeringPID_ratio_kp = 1.8f;//0.85f;
        steeringPID_ratio_kd = 0.6f;
        SightForward_ratio = 0.6f;
        OuterSide_Ratio_ratio = 1.3f;
        InnerSide_Ratio_ratio = 0.9f;
    }


    if (flag_For_Right_Circle==1)
    {
        speed_Target_ratio = 0.95f;
        steeringPID_ratio_kp = 1.8f;//0.85f;
        steeringPID_ratio_kd = 0.6f;
        SightForward_ratio = 0.6f;
        OuterSide_Ratio_ratio = 1.3f;
        InnerSide_Ratio_ratio = 0.9f;
    }

    if (classification_Result==4 && ThreeeRoad_Delay_Flag == 0)
    {
        steeringPID_ratio_kp = 0.05f;
        steeringPID_ratio_kd = 0.05f;
    }

    if (is_Slope==1)
    {
        speed_Target_ratio = SlopeSpeed1;
        SightForward_ratio = 0.5f;
        steeringPID_ratio_kp = 2.2f;
    }
    if (is_Slope==2)
    {
        speed_Target_ratio = SlopeSpeed2;
        SightForward_ratio = 0.5f;
        steeringPID_ratio_kp = 2.2f;
    }
    if (is_Slope==3)
    {
        speed_Target_ratio = SlopeSpeed3;
        SightForward_ratio = 1.0f;
        steeringPID_ratio_kp = 1.1f;
    }
    if (zebra_status == finishing)
    {
        OuterSide_Ratio_ratio = 3.0f;
        InnerSide_Ratio_ratio = 3.0f;
        SightForward_ratio =0.7f;
        steeringPID_ratio_kp = 1.3f;
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
        speed_Status = Lowest;
    }
    switch(speed_Status)
    {
        case Highest:
        {
            SightForward = SightForward_Highest*SightForward_ratio;
            speed_Target = speed_Target_Highest*speed_Target_ratio;
            OuterSide_Ratio = OuterSide_Ratio_Highest*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_Highest*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_Highest[0]*steeringPID_ratio_kp,Steering_PID_Highest[1],Steering_PID_Highest[2]*steeringPID_ratio_kd);
            break;
        }
        case High:
        {
            SightForward = SightForward_High*SightForward_ratio;
            speed_Target = speed_Target_High*speed_Target_ratio;
            OuterSide_Ratio = OuterSide_Ratio_High*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_High*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_High[0]*steeringPID_ratio_kp,Steering_PID_High[1],Steering_PID_High[2]*steeringPID_ratio_kd);
            break;
        }
        case Low:
        {
            SightForward = SightForward_Low*SightForward_ratio;
            speed_Target = speed_Target_Low*speed_Target_ratio;
            OuterSide_Ratio = OuterSide_Ratio_Low*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_Low*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_Low[0]*steeringPID_ratio_kp,Steering_PID_Low[1],Steering_PID_Low[2]*steeringPID_ratio_kd);
            break;
        }
        case Lowest:
        {
            SightForward = SightForward_Lowest*SightForward_ratio;
            speed_Target = speed_Target_Lowest*speed_Target_ratio;
            OuterSide_Ratio = OuterSide_Ratio_Lowest*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_Lowest*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_Lowest[0]*steeringPID_ratio_kp,Steering_PID_Lowest[1],Steering_PID_Lowest[2]*steeringPID_ratio_kd);
            break;
        }
        case Lowest_ForT:
        {
            SightForward = SightForward_Lowest_ForT*SightForward_ratio;
            speed_Target = speed_Target_Lowest_ForT*speed_Target_ratio;
            OuterSide_Ratio = OuterSide_Ratio_Lowest_ForT*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_Lowest_ForT*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_Lowest_ForT[0]*steeringPID_ratio_kp,Steering_PID_Lowest_ForT[1],Steering_PID_Lowest_ForT[2]*steeringPID_ratio_kd);
            break;
        }
        case Lowest_ForZebra:
        {
            SightForward = SightForward_Lowest_ForT*SightForward_ratio;
            speed_Target = 0.85f*Stop;
            OuterSide_Ratio = OuterSide_Ratio_Lowest_ForT*OuterSide_Ratio_ratio;
            InnerSide_Ratio = InnerSide_Ratio_Lowest_ForT*InnerSide_Ratio_ratio;
            Change_Steering_PID(Steering_PID_Lowest_ForT[0]*steeringPID_ratio_kp,Steering_PID_Lowest_ForT[1],Steering_PID_Lowest_ForT[2]*steeringPID_ratio_kd);
            break;
        }
    }

    Cal_Steering_Error(SightForward);
    Cal_Steering_Target();//����ȫ�ֱ����������壩����λ��ʽPDԭ����ת��Ŀ��Steering_Target(��Χ-30~30��������ת��������ת)

}

void Is_Emergency_Stop(void)
{
    if(Check_TRoad(0,0.1f,3) && zebra_status!=starting && is_Slope == 0)
    {
        emergency_Stop=1;
    }
}

void Confirm_ControlMethod(void)
{
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
//                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
//                    {
//                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
//                    }
//                    else
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
//                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
//                    {
//                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
//                    }
//                    else
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
//                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
//                    {
//                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
//                    }
//                    else
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
//                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
//                    {
//                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
//                    }
//                    else
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
//                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
//                    {
//                        PID_mode1 = BANGBANG_CLOSED_LOOP1;
//                    }
//                    else
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
//                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
//                    {
//                        PID_mode2 = BANGBANG_CLOSED_LOOP2;
//                    }
//                    else
                {
                    PID_mode2 = PID_CLOSED_LOOP2;
                }
            }
            break;
        }
    }
}

void camera_thread_entry(void *parameter)
{
    rt_kprintf("camera thread is running.\n");
    //My_Init_Camera();

    while(1)
    {
        if (RT_EOK == rt_sem_take(dma_sem, RT_WAITING_NO))
        {

            mt9v03x_finish_flag = 0;//��ʾ���Ը���mt9v03x_image��
            /*1*/Image_Process();//��ֵ��+��͸��
            /*2*/Update_Left_Unknown_or_Right();//�������ҵ÷�
            /*2*/Check_Special_Classification();//����ʶ���Ӧ�Ĳ���
            /*3*/Update_Timer();
            /*4*/Basic_Classification();//����ʶ��
            /*4*/DrawCenterLine();//����
            /*5*/Confirm_Speed();//ȷ���ٶ�״̬
            /*5*/Confirm_Motion();//�����ٶ�״̬������ؼ���
            /*0*/Is_Emergency_Stop();
        }
        Confirm_ControlMethod();
    }
}

void OLED_thread_entry(void *parameter)
{
    //oled_init();
    rt_kprintf("OLED thread is running.\n");

    while(1)
    {
        rt_thread_mdelay(20);
        Update_OLED_per10ms();
    }
}

void ZEBRA_thread_entry(void *parameter)
{
    rt_kprintf("ZEBRA thread is running.\n");

    while(1)
    {


        rt_thread_mdelay(10);


        //������������
        //            if (steering_Target<=46 && steering_Target>=-46)
        //            if (steering_Error<=300 && steering_Error>=-300 && is_Slope==0)
        Check_Zebra(Zebra_Detect);
        if (Check_Fake_Zebra(3) && (Left_Straight_Score>=3.0f||Unknown_Straight_Score>=3.0f||Right_Straight_Score>=3.0f) && is_Slope==0 && classification_Result!=2 &&classification_Result!=3 &&classification_Result!=4)
        {
            if (White2Black_cnt>=Zebra_Value && White2Black_cnt<=18 && zebra_status == finding)
            {
                Zebra_times++;
                if (Zebra_times<Zebra_times_Max)
                {
                    zebra_status = banning;
                    time_up[11] = 1.0f;
                    Reset_Timer(11);
                    Start_Timer(11);
                }
                else
                {
                    //                        zebra_status = ready_finishing;
                    zebra_status = finishing;
                }
            }
        }
        //            if (zebra_status==ready_finishing)
        //            {
        //                if (White2Black_cnt<=TurnTime)
        //                {
        //                    zebra_status=finishing;
        //                }
        //            }
        if (Read_Timer_Status(11) == RUNNING)
        {
            if (Read_Timer(11)>time_up[11])
            {
                Reset_Timer(11);
                zebra_status = finding;
            }
        }
    }
}

void VL53L0X_thread_entry(void *parameter)
{
    rt_kprintf("VL53L0X thread is running.\n");
//    if (flag_for_ICM_Init == 0)
//    {
//        VL53L0X_Init();
//        //            My_Init_ICM();//�ҵĳ�ʼ��ICM
//        //            Get_Zero_Bias();//����������Ưֵ
//        flag_for_ICM_Init = 1;
//
//    }
    while(1)
    {
        if (classification_Result==2||classification_Result==3)
        {
            Lazer_Data = 819.1f;
        }
        else
        {
            float Lazer_Data_temp = VL53L0X_GetValue()/10.0f;
            if (Lazer_Data_temp>15.0f)//2.1f)
                {
                if (is_Slope!=1 && is_Slope!=3)
                {
                    static uint8 cnt=0;
                    if (Lazer_Data_temp>800.0f)
                    {
                        cnt++;
                        if (cnt>=5)
                        {
                            Lazer_Data = Lazer_Data_temp;
                        }
                    }
                    else
                    {
                        cnt=0;
                        Lazer_Data = Lazer_Data_temp;
                    }
                }
                else
                {
                    Lazer_Data = Lazer_Data_temp;
                }
                }
        }
        if (Lazer_On == 1)
        {
            Check_Slope_with_Lazer();
        }

        rt_thread_mdelay(10);
    }
}

void ADC_thread_entry(void *parameter)
{

//    rt_kprintf("ADC thread is running.\n");
    //My_Init_ADC();
    while(1)
    {
        Get_ADC_DATA();

        rt_thread_mdelay(10);
    }
}

void LED_thread_entry(void *parameter)
{
    rt_kprintf("LED thread is running.\n");
    //My_Init_LED();
    while(1)
    {
        gpio_toggle(P10_5);
        //gpio_toggle(P10_6);

        if(RT_EOK == rt_sem_take(key_sem, RT_WAITING_NO))
        {
            gpio_toggle(P10_6);
        }
        rt_thread_mdelay(500);
    }
}

void KS_timer_entry(void *parameter)
{
        //Update_OLED_per10ms();
        Check_Key_per10ms();
        Check_Switch_per10ms();
}

void motion_timer_entry(void *parameter)
{
    if (emergency_Stop == 0)
    {
        //���ٶȡ�ת��Ƕȵ�Ŀ��ֵ��ͨ��PID���㷨���ı�ֱ������Ͷ����״̬
        if (start_Flag==1)
        {
            Differential_Motor();
        }
        Get_Speed_perSPEED_MEASURING_PERIOD_ms1();
        Get_Speed_perSPEED_MEASURING_PERIOD_ms2();
        Cal_Speed_Output1();
        Cal_Speed_Output2();
        Set_Speed1();
        Set_Speed2();
        Set_Steering();
    }
    else
    {
        start_Flag = 0;
        speed_Target = 0;
        speed_Target1 = 0;
        speed_Target2 = 0;
        //      speed_Output1 = 0;
        //      speed_Output2 = 0;
        steering_Target = 0;
        Get_Speed_perSPEED_MEASURING_PERIOD_ms1();
        Get_Speed_perSPEED_MEASURING_PERIOD_ms2();
        Cal_Speed_Output1();//����
        Cal_Speed_Output2();//����
        Set_Speed1();
        Set_Speed2();
        Set_Steering();
    }
}






int TIMETIME_timer_create (void)
{
    rt_timer_t timer1;
    // ����һ����ʱ�� ����100��tick
    timer1 = rt_timer_create(
        "timer_ms",                                           // timer1��ʾ��ʱ�������ƣ�8���ַ��ڡ�
        Timer_Action_per1ms,                                          // timerout1��ʾʱ�䵽��֮����Ҫִ�еĺ���
        RT_NULL,                                            // RT_NULL��ʾ����Ҫ���ݲ�����
        1,                                                // 100��ʾ��ʱ���ĳ�ʱʱ��Ϊ100��ϵͳtick��ϵͳ����Ϊ1���룬��100��ʾ100����
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC��ʾ��ʱ������������  �������ΪRT_TIMER_FLAG_ONE_SHOT��ֻ������һ��

    // ���ȼ�鶨ʱ�����ƿ鲻�ǿգ���������ʱ��
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }

    return 0;
}

int KS_timer_create (void)
{
    rt_timer_t timer1;
    // ����һ����ʱ�� ����100��tick
    timer1 = rt_timer_create(
        "timer_KS",                                           // timer1��ʾ��ʱ�������ƣ�8���ַ��ڡ�
        KS_timer_entry,                                          // timerout1��ʾʱ�䵽��֮����Ҫִ�еĺ���
        RT_NULL,                                            // RT_NULL��ʾ����Ҫ���ݲ�����
        10,                                                // 100��ʾ��ʱ���ĳ�ʱʱ��Ϊ100��ϵͳtick��ϵͳ����Ϊ1���룬��100��ʾ100����
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC��ʾ��ʱ������������  �������ΪRT_TIMER_FLAG_ONE_SHOT��ֻ������һ��

    // ���ȼ�鶨ʱ�����ƿ鲻�ǿգ���������ʱ��
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }

    return 0;
}

int motion_timer_create(void)
{
    rt_timer_t timer1;
    // ����һ����ʱ�� ����100��tick
    timer1 = rt_timer_create(
        "timer_mot",                                           // timer1��ʾ��ʱ�������ƣ�8���ַ��ڡ�
        motion_timer_entry,                                          // timerout1��ʾʱ�䵽��֮����Ҫִ�еĺ���
        RT_NULL,                                            // RT_NULL��ʾ����Ҫ���ݲ�����
        10,                                                // 100��ʾ��ʱ���ĳ�ʱʱ��Ϊ100��ϵͳtick��ϵͳ����Ϊ1���룬��100��ʾ100����
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC��ʾ��ʱ������������  �������ΪRT_TIMER_FLAG_ONE_SHOT��ֻ������һ��
    // ���ȼ�鶨ʱ�����ƿ鲻�ǿգ���������ʱ��
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    return 0;
}

int LED_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("LED",                      // �߳�����
            LED_thread_entry,                                      // �߳���ں���
        RT_NULL,                                            // �̲߳���
        512,                                                // 512 ���ֽڵ�ջ�ռ�
        20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
                                                            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
        5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create LED thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("LED thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("name thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int ADC_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("ADC",                      // �߳�����
            ADC_thread_entry,                                      // �߳���ں���
        RT_NULL,                                            // �̲߳���
        512,                                                // 512 ���ֽڵ�ջ�ռ�
        20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
                                                            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
        5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create ADC thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("ADC thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("ADC thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int VL53L0X_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("VL53L0X",                      // �߳�����
            VL53L0X_thread_entry,                                      // �߳���ں���
            RT_NULL,                                            // �̲߳���
            512,                                                // 512 ���ֽڵ�ջ�ռ�
            20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
            5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create VL53L0X thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("VL53L0X thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("VL53L0X thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int ZEBRA_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("ZEBRA",                      // �߳�����
            ZEBRA_thread_entry,                                      // �߳���ں���
            RT_NULL,                                            // �̲߳���
            512,                                                // 512 ���ֽڵ�ջ�ռ�
            20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
            5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create ZEBRA thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("ZEBRA thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("ZEBRA thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int OLED_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("OLED",                      // �߳�����
            OLED_thread_entry,                                      // �߳���ں���
            RT_NULL,                                            // �̲߳���
            512,                                                // 512 ���ֽڵ�ջ�ռ�
            20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
            5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create OLED thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("OLED thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("OLED thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int camera_thread_create(void)
{
    mt9v03x_finish_sem = rt_sem_create("mt9v03x_finish_sem", 0 ,RT_IPC_FLAG_FIFO);
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("camera",                      // �߳�����
            camera_thread_entry,                                      // �߳���ں���
            RT_NULL,                                            // �̲߳���
            512,                                                // 512 ���ֽڵ�ջ�ռ�
            20,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
            5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create camera thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("camera thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("camera thread create ERROR.\n");
        return 1;
    }

    return 0;
}



void init_thread_entry(void *parameter)
{
    My_Init_LED();
    LED_thread_create();

    key_sem = rt_sem_create("key_semaphore", 0 ,RT_IPC_FLAG_FIFO);
    dma_sem = rt_sem_create("dma_semaphore", 0 ,RT_IPC_FLAG_FIFO);

    My_Init_Key();
    oled_init();
    My_Init_Switch();
    My_Init_Camera();
    My_Init_SpeedSensor1();//�ҵĳ�ʼ��������
    My_Init_SpeedSensor2();//�ҵĳ�ʼ��������
    My_Init_Steering();//�ҵĳ�ʼ�����
    My_Init_Motor1();//�ҵĳ�ʼ��ֱ�����
    My_Init_Motor2();//�ҵĳ�ʼ��ֱ�����
    My_Init_ADC();
    VL53L0X_Init();



    OLED_thread_create();
    //KEY_thread_create();
    //SWITCH_thread_create();
    KS_timer_create();
    camera_thread_create();
    //motion_thread_create();
    motion_timer_create();
    ZEBRA_thread_create();
    ADC_thread_create();
    VL53L0X_thread_create();

    Get_Inverse_Perspective_Table();//����͸�ӱ�
}

int init_thread_create(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("init",                      // �߳�����
            init_thread_entry,                                      // �߳���ں���
        RT_NULL,                                            // �̲߳���
        512,                                                // 512 ���ֽڵ�ջ�ռ�
        5,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
                                                            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
        5);                                                 // ʱ��ƬΪ5

    rt_kprintf("create init thread.\n");
    if(tid != RT_NULL)                                     // �̴߳����ɹ�
    {
        rt_kprintf("init thread create OK.\n");
        rt_thread_startup(tid);                            // ���и��߳�
    }
    else                                                    // �̴߳���ʧ��
    {
        rt_kprintf("init thread create ERROR.\n");
        return 1;
    }

    return 0;
}




INIT_APP_EXPORT(TIMETIME_timer_create);
INIT_APP_EXPORT(init_thread_create);



int main(void)
{
   // disableInterrupts();
    //    get_clk();//��ȡʱ��Ƶ��  ��ر���
    //�û��ڴ˴����ø��ֳ�ʼ��������
    //    EEPROM_Write_Data(ID_STEERING_DUTY_CENTER, &STEERING_DUTY_CENTER);//��д��ȥ
    //  STEERING_DUTY_CENTER = EEPROM_Read_Data(ID_STEERING_DUTY_CENTER,uint32);//�ٶ�ȡ
//    My_Init_Steering();//�ҵĳ�ʼ�����
//    My_Init_Motor1();//�ҵĳ�ʼ��ֱ�����
//    My_Init_Motor2();//�ҵĳ�ʼ��ֱ�����
//    My_Init_SpeedSensor1();//�ҵĳ�ʼ��������
//    My_Init_SpeedSensor2();//�ҵĳ�ʼ��������
//    My_Init_OLED();//�ҵĳ�ʼ��OLED
//    My_Init_Camera();//�ҵĳ�ʼ������ͷ
//    My_Init_Switch();//�ҵĳ�ʼ������
//    My_Init_Key();//�ҵĳ�ʼ������
//    My_Init_UART();//�ҵĳ�ʼ������ͨ��
//    My_Init_FuzzyPID_Speed();//�ҵĳ�ʼ���ٶ�ģ��PID����
//    My_Init_Timer();//�ҵĳ�ʼ��TIMER

//    My_Init_ADC();//�ҵĳ�ʼ��ADC
//    if (flag_for_ICM_Init == 0)
//    {
//        VL53L0X_Init();
//        //            My_Init_ICM();//�ҵĳ�ʼ��ICM
//        //            Get_Zero_Bias();//����������Ưֵ
//        flag_for_ICM_Init = 1;
//
//    }






    //�ȴ����к��ĳ�ʼ�����
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);






    //��ʼ��LED����

    while(1)
    {
        //��תLED����


        rt_thread_mdelay(500);
    }
}

#pragma section all restore


