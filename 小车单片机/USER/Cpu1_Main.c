/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#include "KEY.h"//按键扫描相关
#include "CAMERA.h"//摄像头、图像处理相关
#include "OLED.h"//显示屏相关
#include "STEERING.h"//舵机相关
#include "UART.h"
#include "fastlz.h"
#include "TIME.h"
#include "SEARCH.h"
#include "MOTOR1.h"//直流电机相关
#include "MOTOR2.h"//直流电机相关
#include "MOTOR_CTL.h"
#include "SWITCH.h"



#pragma section all "cpu1_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等





	//等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //来自摄像机的图像到达后，进行图像处理
        if (mt9v03x_finish_flag == 1 && (UART_Flag_TX == FALSE || UART_EN == FALSE))
        {
            mt9v03x_finish_flag = 0;//表示可以更新mt9v03x_image了
            InsertTimer1Point(0);
            InsertTimer2Point(6);

            //Get_Cutted_Image();//裁剪图像到188*40


            InsertTimer1Point(1);

            Get_ADC_DATA();//更新电压读取
            Get_ICM_DATA();//更新陀螺仪数据

            if (Thresholding_Value_Init_Flag == 0)
            {
                Get_Inverse_Perspective_Table();//求逆透视表
                Get_Thresholding_Value();//求二值化阈值
                time_up[3]=1.0f;
                Start_Timer(3);//启动计时
                Thresholding_Value_Init_Flag = 1;
            }
            Get_Thresholding_Image();

            InsertTimer1Point(2);
            Get_Inverse_Perspective_Image();
            OLED_Camera_flag=1;

            InsertTimer1Point(3);

            //窗口默认处于中下位置
            Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);

            //如果是3右环岛、4三岔路口，且定时器没有在计时，就开定时
            if (Read_Timer_Status(0) == PAUSED)
            {
                switch(classification_Result)
                {
                    case 2://左环岛
                        if (flag_For_Left_Circle == 0)//说明还没进左环岛
                        {
                            flag_For_Left_Circle = 1;
                            //classification_Result = 7;//7靠左
                            time_up[0] = rightCircle_RightTime;
                            Start_Timer(0);
                        }
                        break;
                    case 3://右环岛
                        if (flag_For_Right_Circle == 0)//说明还没进右环岛
                        {
                            flag_For_Right_Circle = 1;
                            //classification_Result = 8;//8靠右
                            time_up[0] = rightCircle_RightTime;
                            Start_Timer(0);
                        }
                        break;
                    case 4:
                        classification_Result = 4;//4三岔路口
                        time_up[0] = threeRoads_RightTime;
                        Start_Timer(0);
                        break;
                    case 10://左直线
                        if (flag_For_Right_Circle == 1) //说明准备出右环岛
                        {
                            flag_For_Right_Circle = 2;
                            //classification_Result = 7;//7靠左
                            time_up[0] = rightCircle_LeftTime;
                            Start_Timer(0);
                            time_up[4] = rightCircle_BannedTime;
                            Start_Timer(4);
                        }
                        break;
                    case 11://右直线
                        if (flag_For_Left_Circle == 1) //说明准备出左环岛
                        {
                            flag_For_Left_Circle = 2;
                            //classification_Result = 8;//8靠右
                            time_up[0] = rightCircle_LeftTime;
                            Start_Timer(0);
                            time_up[5] = rightCircle_BannedTime;
                            Start_Timer(5);
                        }
                        break;
                    case 14://T字
                        if (flag_For_Right_T == 1) //说明准备出右丁字
                        {
                            flag_For_Right_T = 0;
                            classification_Result = 7;//7靠左
                            time_up[0] = T_Time;
                            Start_Timer(0);
                        }
                        else if (flag_For_Left_T == 1)
                        {
                            flag_For_Right_T = 0;
                            classification_Result = 8;//8靠右
                            time_up[0] = T_Time;
                            Start_Timer(0);
                        }
                        break;
                    default:
                        break;
                }

            }
            //如果在计时，判断计时是否达到要求时间
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
            //如果不在计时，继续分类
            if (Read_Timer_Status(0) == PAUSED)
            {
                //小车处于右圆环状态
                if (flag_For_Right_Circle == 1)
                {
                    if (Check_Left_Straight(2,0,1) == 0)
                    {
                        classification_Result = 3;//8;
                    }
                    else
                    {
                        classification_Result = 10;//10左直线
                    }
                }
                //小车处于左圆环状态
                else if (flag_For_Left_Circle == 1)
                {
                    if (Check_Right_Straight(0,-2,1) == 0)
                    {
                        classification_Result = 2;//7;
                    }
                    else
                    {
                        classification_Result = 11;//11右直线
                    }
                }
                else
                {//正常识别
                    if (flag_For_Right_Circle == 2 && Read_Timer_Status(4) == PAUSED)
                    {
                        flag_For_Right_Circle = 0;
                    }
                    if (flag_For_Left_Circle == 2 && Read_Timer_Status(5) == PAUSED)
                    {
                        flag_For_Left_Circle = 0;
                    }
                    if (Check_Straight(1.0f))
                    {
                        classification_Result = 6;//6直道
                    }
                    else
                    {
                        Classification_Classic36(0,&classification_Result,&classification_Result_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        Check(&classification_Result,classification_Result_2nd);
                        Check(&classification_Result,9);
                    }
                    Check_Classification(classification_Result,1);

                    //以下是新窗口的识别
                    Set_Search_Range(0,height_Inverse_Perspective*6/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);
                    if (Check_Straight(0.5f))
                     {
                         classification_Result_1 = 6;//6直道
                     }
                     else
                     {
                         Classification_Classic36(1,&classification_Result_1,&classification_Result_1_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                         Check(&classification_Result_1,classification_Result_1_2nd);
                         Check(&classification_Result_1,9);

                     }
                     Check_Classification(classification_Result_1,1);

                     Set_Search_Range(0,height_Inverse_Perspective,width_Inverse_Perspective/4,width_Inverse_Perspective/2);
                     if (Check_Straight(0.6f))
                     {
                         classification_Result=6;
                         classification_Result_1=6;
                     }
                     //改回默认窗口
                     Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective/2);

                     //检查长直道是否满足
                     if((classification_Result_1==6||classification_Result_1==5) && (classification_Result==6||classification_Result==5))
                     {
                         Long_Straight_Flag = 1;//长直道
                     }
                     else
                     {
                         Long_Straight_Flag = 0;//长直道
                     }
                }
            }
            DrawCenterLine();
//            Compensate_ColCenter();


            //由处理后的图像等信息，获取速度、转向角度的目标值

            //Cal_Steering_Error(Get_d_steering_Error()<30.0f?0.5f:(Get_d_steering_Error()>120.0f?0.55f:((Get_d_steering_Error()-30.0f)/(120.0f-30.0f)*(0.55f-0.5f)+0.5f)));//根据Col_Center和扫描范围search_Lines计算误差（全局变量，待定义）
            //if ((steering_Error>130||steering_Error<-130) && classification_Result == 9)
            //进入条件：识别类型为9或者处于环岛，且误差变化率大的；处于环岛入口阶段的
//            if ((((d_steering_Error>0?d_steering_Error:-d_steering_Error)>30) && (classification_Result == 9 || (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1) ) )  )
//            {
//                Cal_Steering_Error(SightForward);
//                speed_Target = speed_Target_Low;
//
//                //Differential_Ratio = 2.5f;//1.3f;
//
//                Change_Steering_PID(0.25f,0,0.30f);
//                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//进圆环瞬间单独设转向pid
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//                else if (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)
//                {
//                    Change_Steering_PID(0.24f,0,0.20f);
//                }
//                else if (speed_Target_Low >= 2.2f && speed_Target_High >= 2.4f)//只有2.1/1.9以上才可以
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
//                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//进圆环瞬间单独设转向pid
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

            //确定速度状态
            //"0左弯", "1右弯",
            //"2左环岛", "3右环岛",
            //"4三岔路口", "5十字路口",
            //"6直道","7靠左（临时使用）","8靠右（临时使用）",
            //"9未知","10左直线","11右直线",
            //"12左丁字","13右丁字",
            //"14T字"

            uint8 set_flag=0;//表示这次是否设置了速度


            if(Long_Straight_Flag == 1)
            {
                speed_Status = Highest;
                set_flag=1;
                time_up[6] = 0.8f/(speed_Target_Highest);
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
                Reset_Timer(6);
            }


            static uint8 status_switch_1=0;
            if(steering_Target>35||steering_Target<-35)
            {
                status_switch_1=1;
            }
            if (steering_Target<28 && steering_Target>-28)
            {
                status_switch_1=0;
            }
            if (status_switch_1==1)
            {
                speed_Status = Low;
                set_flag=1;
                Reset_Timer(6);
            }



            if(classification_Result == 14)//T字
            {
                speed_Status = Lowest;
                Reset_Timer(6);
                set_flag=1;
                time_up[7] = T_Time;
                Reset_Timer(7);
                Start_Timer(7);
            }
            else if (Read_Timer_Status(7) == RUNNING)
            {
                speed_Status = Lowest;
                set_flag=1;
                if (Read_Timer(7)>time_up[7])
                {
                    Reset_Timer(7);
                }
            }


            static uint8 status_switch_2=0;
            if(d_steering_Error>30||d_steering_Error<-30)
            {
                status_switch_2=1;
            }
            if(d_steering_Error<24&&d_steering_Error>-24)
            {
                status_switch_2=0;
            }
            if (status_switch_2==1)
            {
                speed_Status = Lowest;
                set_flag=1;
                Reset_Timer(6);
            }


            if(set_flag==0)
            {
                speed_Status = High;
                Reset_Timer(6);
            }


            //根据速度状态进行相关计算

            enum SpeedMode speed_Mode_temp = switch_Status[Switch1]+2*switch_Status[Switch2];
            if (speed_Mode_temp!=speed_Mode)
            {
                speed_Mode = speed_Mode_temp;
                Update_Speed_Mode();
            }
            switch(speed_Status)
            {
                case Highest:
                {
                    SightForward = SightForward_Highest;
                    speed_Target = speed_Target_Highest;
                    InnerSide_Ratio = InnerSide_Ratio_Highest;
                    Change_Steering_PID(Steering_PID_Highest[0],Steering_PID_Highest[1],Steering_PID_Highest[2]);
                    break;
                }
                case High:
                {
                    SightForward = SightForward_High;
                    speed_Target = speed_Target_High;
                    InnerSide_Ratio = InnerSide_Ratio_High;
                    Change_Steering_PID(Steering_PID_High[0],Steering_PID_High[1],Steering_PID_High[2]);
                    break;
                }
                case Low:
                {
                    SightForward = SightForward_Low;
                    speed_Target = speed_Target_Low;
                    InnerSide_Ratio = InnerSide_Ratio_Low;
                    Change_Steering_PID(Steering_PID_Low[0],Steering_PID_Low[1],Steering_PID_Low[2]);
                    break;
                }
                case Lowest:
                {
                    SightForward = SightForward_Lowest;
                    speed_Target = speed_Target_Lowest;
                    InnerSide_Ratio = InnerSide_Ratio_Lowest;
                    Change_Steering_PID(Steering_PID_Lowest[0],Steering_PID_Lowest[1],Steering_PID_Lowest[2]);
                    break;
                }
            }
            Cal_Steering_Error(SightForward);



            UART_Flag_TX = TRUE;

            InsertTimer1Point(4);
        }



        //低速目标且低速时，开环
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
        }



    }
}



#pragma section all restore
