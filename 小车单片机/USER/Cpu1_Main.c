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

            Get_Cutted_Image();//裁剪图像到188*40


            InsertTimer1Point(1);

            //Get_ICM_DATA();//更新陀螺仪数据

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

            InsertTimer1Point(3);



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
                    if (Check_Straight())
                    {
                        classification_Result = 6;//6直道
                    }
                    else
                    {
                        classification_Result = Classification_Classic36();//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        if (classification_Result == 13)//右丁字
                        {
                            flag_For_Right_T = 1;
                            flag_For_Left_T = 0;
                        }
                        if (classification_Result == 12)//左丁字
                        {
                            flag_For_Right_T = 0;
                            flag_For_Left_T = 1;
                        }
                        if (classification_Result ==3)//3右环岛
                        {
                            if(flag_For_Right_Circle!=0 || !Check_RightCircle_New())
                            {
                                classification_Result = 9;//9未知
                            }
                        }
                        if (classification_Result ==2)//2左环岛
                        {
                            if(flag_For_Left_Circle!=0 || !Check_LeftCircle_New())
                            {
                                classification_Result = 9;//9未知
                            }
                        }
                        if (classification_Result ==4)//4三岔路口
                        {
                            if(!Check_ThreeRoads_New())
                            {
                                classification_Result = 9;//9未知
                            }
                        }
                        if (classification_Result == 9)//9未知
                        {
                            if(Check_Left_Straight(2,-2,1))
                            {
                                classification_Result = 7;//7靠左
                            }
                            if(Check_Right_Straight(2,-2,1))
                            {
                                classification_Result = 8;//8靠右
                            }
                        }

                    }
                    Check_Classification(classification_Result,1);
                }

            }

            DrawCenterLine();


            //由处理后的图像等信息，获取速度、转向角度的目标值

            //Cal_Steering_Error(Get_d_steering_Error()<30.0f?0.5f:(Get_d_steering_Error()>120.0f?0.55f:((Get_d_steering_Error()-30.0f)/(120.0f-30.0f)*(0.55f-0.5f)+0.5f)));//根据Col_Center和扫描范围search_Lines计算误差（全局变量，待定义）
            //if ((steering_Error>130||steering_Error<-130) && classification_Result == 9)
            //进入条件：识别类型为9或者处于环岛，且误差变化率大的；处于环岛入口阶段的
            if (((Get_d_steering_Error()>30) && (classification_Result == 9 || (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1) ) )  ||  (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)) )
            {
                Cal_Steering_Error(0.55);
                speed_Target = speed_Target_Min;

                Differential_Ratio = 1.3f;
//                if (speed_Target_Min >= 2.4f && speed_Target_Max >= 2.6f)//只有2.3/2.1以上才可以
//                {
//                    Differential_Ratio = 1.1f;
//                }
//                if (speed_Target_Min >= 2.8f && speed_Target_Max >= 3.2f)//只有2.8/2.5以上才可以
//                {
//                    Differential_Ratio = 1.1f;
//                }


                Change_Steering_PID(0.25f,0,0.30f);
                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//进圆环瞬间单独设转向pid
                {
                    if (flag_For_Right_Circle == 1)
                    {
                        steering_Error =  300;
                    }
                    if (flag_For_Left_Circle == 1)
                    {
                        steering_Error =  -300;
                    }
                    Change_Steering_PID(0.4f,0,0.30f);
                }
                else if (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)
                {
                    Change_Steering_PID(0.20f,0,0.20f);
                }
                else
                {
                    if (speed_Target_Min >= 2.2f && speed_Target_Max >= 2.4f)//只有2.1/1.9以上才可以
                    {
                        Change_Steering_PID(0.27f,0,0.30f);
                    }
                }
            }
            else
            {
                Cal_Steering_Error(0.55);
                speed_Target = speed_Target_Max;

                Differential_Ratio = 1.3f;
//                if (speed_Target_Min >= 2.4f && speed_Target_Max >= 2.6f)//只有2.3/2.1以上才可以
//                {
//                    Differential_Ratio = 1.1f;
//                }

                Change_Steering_PID(0.25f,0,0.30f);
                if (Read_Timer_Status(0) == RUNNING && (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1))//进圆环瞬间单独设转向pid
                {
                    Change_Steering_PID(0.4f,0,0.30f);
                }
                else if (flag_For_Right_Circle == 1 || flag_For_Left_Circle == 1)
                {
                    Change_Steering_PID(0.20f,0,0.20f);
                }

            }



            UART_Flag_TX = TRUE;

            InsertTimer1Point(4);
        }



        //低速目标且低速时，开环
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



    }
}



#pragma section all restore
