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
#include "LED.h"
#include "ICM.h"
#include "BEEP.h"



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


            if (Thresholding_Value_Init_Flag == 0)
            {

                Get_Inverse_Perspective_Table();//求逆透视表
                //Get_Thresholding_Value();//求二值化阈值
                GetBinThreshold_OSTU();//大津法二值化
                time_up[3]=0.5f;
                Start_Timer(3);//启动计时
                Thresholding_Value_Init_Flag = 1;
            }
            Get_Thresholding_Image();




            InsertTimer1Point(2);
            Get_Inverse_Perspective_Image();
            OLED_Camera_flag=1;

            InsertTimer1Point(3);

            //窗口默认处于中下位置
            Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
            Helper_Window_Flag = 0;

            //计算左中右的笔直程度
            Select_Left_Unknown_or_Right(9);
            Check_RoadWidth();

            if (Read_Timer_Status(4) == RUNNING)
            {
                if (Read_Timer(4)>time_up[4])
                {
                    Reset_Timer(4);
                }
            }

            //检查斑马线数据
//            if (steering_Target<=46 && steering_Target>=-46)
//            if (steering_Error<=300 && steering_Error>=-300 && is_Slope==0)
            Check_Zebra(Zebra_Detect);
            if (Check_Fake_Zebra(3) && (Left_Straight_Score>=3.0f||Unknown_Straight_Score>=3.0f||Right_Straight_Score>=3.0f) && is_Slope==0 && classification_Result!=2 &&classification_Result!=3 &&classification_Result!=4 &&classification_Result!=5 &&classification_Result!=12 &&classification_Result!=13)
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


            //激光测距开关时间区
            static uint8 first_flag = 0;
            if (start_Flag == 1 && first_flag==0)
            {
                first_flag = 1;
                time_up[15] = Lazer_Start_Time;
                Start_Timer(15);
                time_up[16] = Lazer_End_Time;
                Start_Timer(16);
            }
            if (Read_Timer_Status(15) == RUNNING)
            {
                if (Read_Timer(15)>time_up[15])
                {
                    Reset_Timer(15);
                    Lazer_Data=819.1f;
                    Lazer_On = 1;
                }
            }
            if (Read_Timer_Status(16) == RUNNING)
            {
                if (Read_Timer(16)>time_up[16])
                {
                    Reset_Timer(16);
                    Lazer_Data=819.1f;
                    Lazer_On = 0;
                }
            }

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
//                        if (flag_For_ThreeRoad==0)
//                        {
//                            classification_Result = 4;//4三岔路口
//                            flag_For_ThreeRoad = 1;
//                        }
                        if (flag_For_ThreeRoad==0||flag_For_ThreeRoad==1)
                        {
                            flag_For_ThreeRoad = 1;
                            time_up[18] = threeRoads_RightTime;
                            Reset_Timer(18);
                            Start_Timer(18);
                        }
                        break;
                    case 5:
                        classification_Result = 5;//5十字路口
//                        time_up[0] = 0.1f;
//                        Start_Timer(0);
                        break;
                    case 10://左直线
                        if (flag_For_Right_Circle == 2) //说明准备出右环岛
                        {
                            flag_For_Right_Circle = 3;
                            //classification_Result = 7;//7靠左
//                            time_up[0] = rightCircle_LeftTime;
//                            Start_Timer(0);
                            time_up[4] = rightCircle_BannedTime;
                            Start_Timer(4);
                        }
                        break;
                    case 11://右直线
                        if (flag_For_Left_Circle == 2) //说明准备出左环岛
                        {
                            flag_For_Left_Circle = 3;
                            //classification_Result = 8;//8靠右
//                            time_up[0] = rightCircle_LeftTime;
//                            Start_Timer(0);
                            time_up[5] = rightCircle_BannedTime;
                            Start_Timer(5);
                        }
                        break;
                    case 14://T字
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


            if (classification_Result!=4 && flag_For_ThreeRoad == 1 && Read_Timer(18)>=ThreeeRoad_Delay && Read_Timer_Status(18) == RUNNING)
            {
                flag_For_ThreeRoad = 2;
            }
            else if (flag_For_ThreeRoad == 1 && Read_Timer_Status(18) == RUNNING)
            {
                flag_For_ThreeRoad = 1;
            }
            else if (Read_Timer_Status(18) == PAUSED)
            {
                flag_For_ThreeRoad = 0;
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
            if (Read_Timer_Status(7) == RUNNING)
            {
                if (Read_Timer(7)>time_up[7])
                {
                    Reset_Timer(7);
                }
            }
            if (Read_Timer_Status(13) == RUNNING)
            {
                if (Read_Timer(13)>time_up[13])
                {
                    Reset_Timer(13);
                }
            }
            if (Read_Timer_Status(14) == RUNNING)
            {
                if (Read_Timer(14)>time_up[14])
                {
                    Reset_Timer(14);
                }
            }
            if (Read_Timer_Status(18) == RUNNING)
            {
                if (Read_Timer(18)>time_up[18])
                {
                    Reset_Timer(18);
                }
            }
            //如果不在计时，继续分类
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
                //小车处于右圆环状态
                else if (flag_For_Right_Circle == 2)
                {
                    Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (Check_Left_Straight(2,0,1) == 0 || Read_Timer_Status(13) == RUNNING)
//                    if(Left_Straight_Score<=5.5f)
                    {
                        classification_Result = 3;//8;
                    }
                    else
                    {
                        classification_Result = 10;//10左直线
                    }
                    Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                }
                //小车处于左圆环状态
                else if (flag_For_Left_Circle == 2)
                {
                    Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (Check_Right_Straight(0,-2,1) == 0 || Read_Timer_Status(13) == RUNNING)
//                    if(Right_Straight_Score<=5.5f)
                    {
                        classification_Result = 2;//7;
                    }
                    else
                    {
                        classification_Result = 11;//11右直线
                    }
                    Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                }
                else if (flag_For_T == 1)
                {
                    classification_Result = 14;
//                    if (Check_TRoad(1,0.18f,3) == 1)
                    if (Check_TRoad(1,T_Line-1.0f/6.0f,3) == 1)
                    {
                        flag_For_T = 2;
                        time_up[7] = 0.2f;
                        Start_Timer(7);
                        time_up[14] = 0.5f;
                        Start_Timer(14);
                    }
                }
                else if (flag_For_T == 2)
                {
//                    if (Read_Timer_Status(7) == PAUSED)
                    if (((Left_Straight_Score>=2.6f||Unknown_Straight_Score>=2.6f||Right_Straight_Score>=2.6f)  && Read_Timer(7)==PAUSED) || Read_Timer(14)==PAUSED)
                    {
                        flag_For_T=0;
                    }
                    else
                    {
                        if (flag_For_Right_T == 1)
                        {
                            classification_Result = 7;//靠左
                            flag_For_Right_T = 0;
                        }
                        if (flag_For_Left_T == 1)
                         {
                             classification_Result = 8;//靠右
                             flag_For_Left_T = 0;
                         }
                    }
                }
                else if (flag_For_ThreeRoad == 2)
                {
                    classification_Result = 4;
                }
                else
                {//正常识别
                    if (flag_For_Right_Circle == 3 && Read_Timer_Status(4) == PAUSED)
                    {
                        flag_For_Right_Circle = 0;
                    }
                    if (flag_For_Left_Circle == 3 && Read_Timer_Status(5) == PAUSED)
                    {
                        flag_For_Left_Circle = 0;
                    }
//                    if (flag_For_ThreeRoad == 2)
//                    {
//                        flag_For_ThreeRoad = 0;
//                    }
                    if (/*Check_Straight(1.0f)*/straight_Alarm == 1||short_straight_Alarm == 1)
                    {
                        classification_Result = 6;//6直道
                    }
                    else if (is_Slope !=0)
                    {
                        classification_Result = 9;
                        Check(&classification_Result,9);
                    }
                    else if (crossRoad_Alarm==1)
                    {
                        classification_Result = 5;
                        Helper_Window_Flag = 0;
                    }
                    else
                    {
                        Classification_Classic36(0,&classification_Result,&classification_Result_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        Check(&classification_Result,classification_Result_2nd);
                        Check(&classification_Result,9);
                    }
//                    Check_Classification(classification_Result,1);

                    //以下是新窗口的识别
                    Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (/*Check_Straight(0.5f)*/straight_Alarm == 1||short_straight_Alarm == 1)
                     {
                         classification_Result_1 = 6;//6直道
                     }
                    else if (is_Slope != 0)
                    {
                        classification_Result_1 = 9;
                        Check(&classification_Result_1,9);
                    }
                    else if (crossRoad_Alarm==1)
                    {
                        classification_Result_1 = 5;
                        Helper_Window_Flag = 0;
                    }
                     else
                     {
                         Classification_Classic36(1,&classification_Result_1,&classification_Result_1_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                         Check(&classification_Result_1,classification_Result_1_2nd);
                         Check(&classification_Result_1,9);

                     }

                     //以下是辅助窗口的识别
                     Set_Search_Range(height_Inverse_Perspective*2/10,height_Inverse_Perspective*8/10-height_Inverse_Perspective*2/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                     if (/*Check_Straight(0.5f)*/straight_Alarm == 1||short_straight_Alarm == 1)
                      {
                          classification_Result_2 = 6;//6直道
                      }
                     else if (is_Slope != 0)
                     {
                         classification_Result_2 = 9;
                         Check(&classification_Result_2,9);
                     }
                     else if (crossRoad_Alarm==1)
                     {
                         classification_Result_2 = 5;
                         Helper_Window_Flag = 0;
                     }
                      else
                      {
                          Classification_Classic36(2,&classification_Result_2,&classification_Result_2_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                          Check(&classification_Result_2,classification_Result_2_2nd);
                          Check(&classification_Result_2,9);
                      }


//                     Set_Search_Range(0,height_Inverse_Perspective,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
//                     if (/*Check_Straight(0.65f)||*/straight_Alarm == 1)
//                     {
//                         classification_Result=6;
//                         classification_Result_1=6;
//                     }
//                     //改回默认窗口
//                     Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);

                     //检查长直道是否满足
//                     if((classification_Result_1==6||classification_Result_1==5) && (classification_Result==6))
//                     {
//                         Long_Straight_Flag = 1;//长直道
//                     }
//                     else
//                     {
//                         Long_Straight_Flag = 0;//长直道
//                     }


                     // 辅助窗口的作用
                     if (classification_Result_2==2||classification_Result_2==3||classification_Result_2==5||classification_Result_2==4||classification_Result_2==12||classification_Result_2==13)
                     {
                         if (speed_Mode == Low_Mode&&(classification_Result_2==2||classification_Result_2==3))
                         {
                             ;
                         }
                         else
                         {
                             if (classification_Result == 7 || classification_Result == 8 || classification_Result == 9)
                             {
                                 classification_Result = classification_Result_2;
                                 Helper_Window_Flag = 2;
                             }
                             else
                             {
                                 Helper_Window_Flag = 0;
                             }
                         }
                     }
                     // 新窗口的作用
                     if (classification_Result_1==12||classification_Result_1==13||classification_Result_1==14)
                     {

                         classification_Result = classification_Result_1;
                         Helper_Window_Flag = 1;

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



            //LED指示
            if(Read_Timer_Status(0) == RUNNING || zebra_status == banning || zebra_status == finishing || is_Slope==2 || is_Slope==3)
            {
                LED_ON(1);
            }
            else
            {
                LED_OFF(1);
            }

            //LED指示
            if(is_Slope==1 || zebra_status == banning || zebra_status == finishing || is_Slope==3)
            {
                LED_ON(2);
            }
            else
            {
                LED_OFF(2);
            }




            //确定速度状态
            //"0左弯", "1右弯",
            //"2左环岛", "3右环岛",
            //"4三岔路口", "5十字路口",
            //"6直道","7靠左（临时使用）","8靠右（临时使用）",
            //"9未知","10左直线","11右直线",
            //"12左丁字","13右丁字",
            //"14T字"

            uint8 set_flag=0;//表示这次是否设置了速度




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
            if(/*d_steering_Error>50||d_steering_Error<-50||*/steering_Target>50||steering_Target<-50)
            {
                status_switch_2=1;
            }
            if(steering_Target<15&&steering_Target>-15)
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

            if(classification_Result == 12||classification_Result == 13 ||classification_Result == 5||classification_Result == 7||classification_Result == 8)
            {
                speed_Status = Low;
                Reset_Timer(6);
                set_flag=1;
                time_up[8] = 2.0f;
                Reset_Timer(8);
                Start_Timer(8);
            }
            else if (Read_Timer_Status(8) == RUNNING)
            {
                speed_Status = Low;
                set_flag=1;
                if (Read_Timer(8)>time_up[8])
                {
                    Reset_Timer(8);
                }
            }


            if(flag_For_T == 1)//T字
            {
                speed_Status = Lowest_ForT;
                Reset_Timer(6);
                set_flag=1;
            }
            else if(flag_For_T == 2)//T字
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

            if (short_straight_Alarm == 1)
            {
                speed_Status = High;
                Reset_Timer(6);
                set_flag=1;
            }

            if(straight_Alarm == 1)
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

            if(set_flag==0)
            {
                speed_Status = Low;
                Reset_Timer(6);
            }


            //根据速度状态进行相关计算

            float steeringPID_ratio_kp = 1.0f;
            float steeringPID_ratio_kd = 1.0f;
            float SightForward_ratio = 1.0f;
            float OuterSide_Ratio_ratio = 1.0f;
            float InnerSide_Ratio_ratio = 1.0f;
            float speed_Target_ratio = 1.0f;

            if(flag_For_T == 2)//T字
            {
                speed_Target_ratio = 0.9f;
            }

//            if (flag_For_Left_Circle == 2)
//            {
//                speed_Target_ratio = 0.95f;
//                steeringPID_ratio_kp = 2.3f;//0.85f;
//                steeringPID_ratio_kd = 0.6f;
//                SightForward_ratio = 0.6f;
//                OuterSide_Ratio_ratio = 1.5f;
//                InnerSide_Ratio_ratio = 1.1f;
//            }
//
//            if (flag_For_Left_Circle == 1)
//            {
//                speed_Target_ratio = 0.95f;
//                steeringPID_ratio_kp = 2.3f;//0.85f;
//                steeringPID_ratio_kd = 0.6f;
//                SightForward_ratio = 0.6f;
//                OuterSide_Ratio_ratio = 1.5f;
//                InnerSide_Ratio_ratio = 1.1f;
//            }
//
//
//            //右圆环奇怪的偏小
//
//            if (flag_For_Right_Circle == 2)
//            {
//                speed_Target_ratio = 0.95f;
//                steeringPID_ratio_kp = 1.8f;//0.85f;
//                steeringPID_ratio_kd = 0.6f;
//                SightForward_ratio = 0.6f;
//                OuterSide_Ratio_ratio = 1.3f;
//                InnerSide_Ratio_ratio = 0.9f;
//            }
//
//
//            if (flag_For_Right_Circle==1)
//            {
//                speed_Target_ratio = 0.95f;
//                steeringPID_ratio_kp = 1.8f;//0.85f;
//                steeringPID_ratio_kd = 0.6f;
//                SightForward_ratio = 0.6f;
//                OuterSide_Ratio_ratio = 1.3f;
//                InnerSide_Ratio_ratio = 0.9f;
//            }


//            if(classification_Result == 5)
//            {
//                speed_Target_ratio = 0.1f;
//            }


            if (Read_Timer_Status(17) == RUNNING)
            {
                if (Read_Timer(17)>time_up[17] || classification_Result == 2 || classification_Result == 3)
                {
                    leftCircle_Alarm = 0;
                    rightCircle_Alarm = 0;
                    Reset_Timer(17);
                }
            }
            else
            {
                if (leftCircle_Alarm == 1)
                {
                    time_up[17] = 1.0f;
                    Reset_Timer(17);
                    Start_Timer(17);
                }
                if (rightCircle_Alarm == 1)
                {
                    time_up[17] = 1.0f;
                    Reset_Timer(17);
                    Start_Timer(17);
                }
            }

            if (rightCircle_Alarm==1||leftCircle_Alarm==1)
            {
                if (rightCircle_Size==1||leftCircle_Size==1)//大圆
                {
                    speed_Target_ratio = 0.9f;
                }
                if (rightCircle_Size==2||leftCircle_Size==2||leftCircle_Size==0||rightCircle_Size==0)//小圆
                {
                    speed_Target_ratio = 0.85f;
                }
            }


            if (leftCircle_Size==1)//大圆
            {
                if (flag_For_Left_Circle == 2)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.3f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 0.9f;
                    InnerSide_Ratio_ratio = 0.7f;
                }

                if (flag_For_Left_Circle == 1)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.45f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 0.9f;
                    InnerSide_Ratio_ratio = 0.7f;
                }
            }
            else if (leftCircle_Size==2||leftCircle_Size==0)//小圆
            {
                if (flag_For_Left_Circle == 2)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.9f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 1.3f;
                    InnerSide_Ratio_ratio = 0.7f;
                }

                if (flag_For_Left_Circle == 1)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 2.8f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 1.3f;
                    InnerSide_Ratio_ratio = 0.7f;
                }
            }


            if (rightCircle_Size==1)//大圆
            {
                //右圆环奇怪的偏小

                if (flag_For_Right_Circle == 2)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.3f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 0.9f;
                    InnerSide_Ratio_ratio = 0.7f;
                }

                if (flag_For_Right_Circle == 1)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.45f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 0.9f;
                    InnerSide_Ratio_ratio = 0.7f;
                }
            }
            else if(rightCircle_Size==2||rightCircle_Size==0)//小圆
            {
                if (flag_For_Right_Circle == 2)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 1.9f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 1.3f;
                    InnerSide_Ratio_ratio = 0.7f;
                }


                if (flag_For_Right_Circle==1)
                {
                    speed_Target_ratio = 0.9f;
                    SightForward_ratio = 0.6f;
                    steeringPID_ratio_kp = 2.8f;
                    steeringPID_ratio_kd = 0.2f;
                    OuterSide_Ratio_ratio = 1.3f;
                    InnerSide_Ratio_ratio = 0.7f;
                }
            }

//            if (classification_Result==2||classification_Result==3)
//            {
//                SightForward_ratio = 0.4f;
//                steeringPID_ratio_kp = 2.4f;
//                steeringPID_ratio_kd = 2.4f;
//            }

            if (flag_For_ThreeRoad == 1)
            {
                SightForward_ratio = 0.6f;
                steeringPID_ratio_kp = 1.5f;
                steeringPID_ratio_kd = 1.5f;
            }


            static uint8 last_classification = 0;
            static uint8 OuterDecline_flag = 0;
            static int cnt_temp=0;
            if (classification_Result==4||((last_classification == 6 || last_classification == 5 ||last_classification == 12||last_classification == 13) && (classification_Result!=6 && classification_Result!=5 && classification_Result!=12&& classification_Result!=13)))
            {
                cnt_temp=0;
                OuterDecline_flag = 1;
//                speed_Target_ratio = 0.1f;
                OuterSide_Ratio_ratio = 0.77f;
                InnerSide_Ratio_ratio = 1.1f;
            }
            if(OuterDecline_flag==1)
            {
//                speed_Target_ratio = 0.1f;
                OuterSide_Ratio_ratio = 0.77f;
                InnerSide_Ratio_ratio = 1.1f;
            }
            if (classification_Result!=6 && classification_Result!=5 && classification_Result!=12 && classification_Result!=13 && classification_Result!=4)
            {
                cnt_temp++;
                if (cnt_temp>25)
                {
                    OuterDecline_flag=0;
                    cnt_temp=0;
                }
            }


            static int last_zebra_status = 0;
            //蜂鸣器指示
            if (last_classification!=classification_Result
                    && classification_Result!=7
                    && classification_Result!=8
                    && classification_Result!=9
                    && zebra_status!=starting)
            {
                BEEP(0.2f,1);
            }
            if (last_zebra_status!=zebra_status)
            {
                BEEP(0.1f,2);
            }
            BEEP_Action();

            last_zebra_status = zebra_status;
            last_classification = classification_Result;




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
                OuterSide_Ratio_ratio = 5.0f;
                InnerSide_Ratio_ratio = 5.0f;
                SightForward_ratio =0.7f;
                steeringPID_ratio_kp = 2.5f;
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
            if (classification_Result==4)
            {
                speed_Status = Low;
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
                    speed_Target = 1.35f*Stop;
                    OuterSide_Ratio = OuterSide_Ratio_Lowest_ForT*OuterSide_Ratio_ratio;
                    InnerSide_Ratio = InnerSide_Ratio_Lowest_ForT*InnerSide_Ratio_ratio;
                    Change_Steering_PID(Steering_PID_Lowest_ForT[0]*steeringPID_ratio_kp,Steering_PID_Lowest_ForT[1],Steering_PID_Lowest_ForT[2]*steeringPID_ratio_kd);
                    break;
                }
            }
            Cal_Steering_Error(SightForward);
            Cal_Steering_Target();//由误差（全局变量，待定义）根据位置式PD原理求转向目标Steering_Target(范围-30~30，负数左转，正数右转)

            static int cnt = 0;
            if (Real_Volt<7.0f)
            {
                cnt++;
            }
            else
            {
                cnt=0;
            }
            if((Check_TRoad(0,0.1f-1.0f/6.0f,3) && zebra_status!=starting && is_Slope == 0)||cnt>100)
            {
                emergency_Stop=1;
            }

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
                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
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
                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
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
                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
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
                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
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
                    if (speed_Measured1 > BANGBANG_UP1 + speed_Target1 || speed_Measured1 < -BANGBANG_DOWN1 + speed_Target1)
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
                    if (speed_Measured2 > BANGBANG_UP2 + speed_Target2 || speed_Measured2 < -BANGBANG_DOWN2 + speed_Target2)
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
