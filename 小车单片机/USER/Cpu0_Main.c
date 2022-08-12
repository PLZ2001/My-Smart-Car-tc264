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
#include "rtthread.h"
#include "CAMERA.h"//摄像头、图像处理相关
#include "SWITCH.h"
#include "KEY.h"//按键扫描相关
#include "OLED.h"//显示屏相关
#include "STEERING.h"//舵机相关
#include "UART.h"//串口通信相关
#include "fastlz.h"//压缩算法
#include "fuzzy_PID.h"//模糊PID算法
#include "SEARCH.h"
#include "WIFI.h"
#include "MOTOR1.h"//直流电机相关
#include "MOTOR2.h"//直流电机相关
#include "ICM.h"
#include "ADC.h"
#include "LED.h"
#include "MOTOR_CTL.h"
#include "TIMETIME.h"
#include "RT_HELPER.h"

#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// ================运行移植提示================
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ================运行移植提示================


//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。
//函数声明


void Image_Process(void)
{
//    if(print_flag==0)
//    {
//        print_flag=1;
//    rt_enter_critical();
//        rt_kprintf("%d_start_ImageProcess:%d\n", flag_count,rt_tick_get());
//        rt_exit_critical();
//        print_flag=0;
//    }
    start_thread[1]=rt_tick_get();
    Get_Thresholding_Image();
    Get_Inverse_Perspective_Image();
    OLED_Camera_flag=1;
//    if(print_flag==0)
//    {
//        print_flag=1;
//    rt_enter_critical();
//        rt_kprintf("%d_end_ImageProcess:%d\n", flag_count,rt_tick_get());
//        rt_exit_critical();
//        print_flag=0;
    end_thread[1]=rt_tick_get();
//    }
}

void Update_Left_Unknown_or_Right(void)
{
    rt_uint32_t e;
    while(1)
    {
        if(rt_event_recv
                (event,                                                 // 事件控制块
                        (EVENT_FLAG2),                            // 事件标志3和事件标志5
                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
                        RT_WAITING_FOREVER,                                     // 一直等待
                        &e) == RT_EOK)
        {
//            if(print_flag==0)
//                {
//                    print_flag=1;
//            rt_enter_critical();
//                    rt_kprintf("%d_start_Update:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
//                    print_flag=0;
//                }
            start_thread[2]=rt_tick_get();
            //计算左中右的笔直程度
            Select_Left_Unknown_or_Right(9);
//            if(print_flag==0)
//                {
//                    print_flag=1;
//            rt_enter_critical();
//                    rt_kprintf("%d_end_Update:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
//                    print_flag=0;
//                }
            end_thread[2]=rt_tick_get();
            rt_event_send(event, EVENT_FLAG4);

        }
    }
}

void Check_Special_Classification(void)
{
    rt_uint32_t e;
    while(1)
    {
        if(rt_event_recv
                (event,                                                 // 事件控制块
                        (EVENT_FLAG3),                            // 事件标志3和事件标志5
                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
                        RT_WAITING_FOREVER,                                     // 一直等待
                        &e) == RT_EOK)
        {
//            if(print_flag==0)
//                {
//                    print_flag=1;
            start_thread[3]=rt_tick_get();
//            rt_enter_critical();
//                    rt_kprintf("%d_start_Special:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
//                    print_flag=0;
//                }
            //如果是3右环岛、4三岔路口，且定时器0没有在计时，就开定时
            if (Read_Timer_Status(0) == PAUSED)
            {
                switch(classification_Result)
                {
                    case 2://左环岛
                        if (flag_For_Left_Circle == 0)//说明还没进左环岛
                        {
                            flag_For_Left_Circle = 1;
                            time_up[0] = rightCircle_RightTime;
                            Start_Timer(0);
                        }
                        break;
                    case 3://右环岛
                        if (flag_For_Right_Circle == 0)//说明还没进右环岛
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
                        classification_Result = 5;//5十字路口
                        break;
                    case 10://左直线
                        if (flag_For_Right_Circle == 2) //说明准备出右环岛
                        {
                            flag_For_Right_Circle = 3;
                            time_up[4] = rightCircle_BannedTime;
                            Start_Timer(4);
                        }
                        break;
                    case 11://右直线
                        if (flag_For_Left_Circle == 2) //说明准备出左环岛
                        {
                            flag_For_Left_Circle = 3;
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
            //如果在计时，判断计时是否达到要求时间
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
            Update_Timer();
//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_end_Special:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
//                print_flag=0;
            end_thread[3]=rt_tick_get();
//            }
            rt_event_send(event, EVENT_FLAG5);
        }

    }

}

void Basic_Classification(void)
{
    rt_uint32_t e;
    while(1)
    {
        if(rt_event_recv
                (event,                                                 // 事件控制块
                        (EVENT_FLAG6),                            // 事件标志3和事件标志5
                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
                        RT_WAITING_FOREVER,                                     // 一直等待
                        &e) == RT_EOK)
        {

//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_start_Basic:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
            start_thread[4]=rt_tick_get();
//                print_flag=0;
//            }
            rt_mutex_take(search_range_mutex, RT_WAITING_FOREVER);
            //窗口默认处于中下位置
            Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
            Helper_Window_Flag = 0;
            //如果不在计时，继续分类
            uint8 classification_Result_TEMP=classification_Result;
            if (Read_Timer_Status(0) == PAUSED)
            {
                if (flag_For_Right_Circle == 1)
                {
                    rt_mutex_release(search_range_mutex);
                    if (Left_Straight_Score<=2.7f)
                    {
                        flag_For_Right_Circle = 2;
                        time_up[13] = 0.75f;
                        Start_Timer(13);
                    }
                }
                else if (flag_For_Left_Circle == 1)
                {
                    rt_mutex_release(search_range_mutex);
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
                    rt_mutex_release(search_range_mutex);
                    if (Check_Left_Straight(2,0,1) == 0 || Read_Timer_Status(13) == RUNNING)
                    {
                        classification_Result_TEMP = 3;//8;
                    }
                    else
                    {
                        classification_Result_TEMP = 10;//10左直线
                    }
                }
                //小车处于左圆环状态
                else if (flag_For_Left_Circle == 2)
                {
                    rt_mutex_release(search_range_mutex);
                    if (Check_Right_Straight(0,-2,1) == 0 || Read_Timer_Status(13) == RUNNING)
                    {
                        classification_Result_TEMP = 2;//7;
                    }
                    else
                    {
                        classification_Result_TEMP = 11;//11右直线
                    }
                }
                else if (flag_For_T == 1)
                {
                    rt_mutex_release(search_range_mutex);
                    classification_Result_TEMP = 14;
                    if (Check_TRoad(1,T_Line,3) == 1)
                    {
                        flag_For_T = 2;
                        time_up[7] = 0.2f;
                        Start_Timer(7);
                        time_up[14] = 0.5f;//0.3f;
                        Start_Timer(14);
                    }
                }
                else if (flag_For_T == 2)
                {
                    rt_mutex_release(search_range_mutex);
                    if (((Left_Straight_Score>=2.6f||Unknown_Straight_Score>=2.6f||Right_Straight_Score>=2.6f)  && Read_Timer(7)==PAUSED) || Read_Timer(14)==PAUSED)
                    {
                        flag_For_T=0;
                    }
                    else
                    {
                        if (flag_For_Right_T == 1)
                        {
                            classification_Result_TEMP = 7;//靠左
                            flag_For_Right_T = 0;
                        }
                        if (flag_For_Left_T == 1)
                        {
                            classification_Result_TEMP = 8;//靠右
                            flag_For_Left_T = 0;
                        }
                    }
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
                    if (Check_Straight(1.0f))
                    {
                        classification_Result_TEMP = 6;//6直道
                    }
                    else if (is_Slope == 1 || is_Slope == 2)
                    {
                        classification_Result_TEMP = 9;
                        Check(&classification_Result_TEMP,9);
                    }
                    else
                    {
                        Classification_Classic36(0,&classification_Result_TEMP,&classification_Result_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        Check(&classification_Result_TEMP,classification_Result_2nd);
                        Check(&classification_Result_TEMP,9);
                    }

                    //以下是新窗口的识别
                    Set_Search_Range(0,height_Inverse_Perspective*6/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (Check_Straight(0.5f))
                    {
                        classification_Result_1 = 6;//6直道
                    }
                    else if (is_Slope != 0)
                    {
                        classification_Result_TEMP = 9;
                        Check(&classification_Result_TEMP,9);
                    }
                    else
                    {
                        Classification_Classic36(1,&classification_Result_1,&classification_Result_1_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        Check(&classification_Result_1,classification_Result_1_2nd);
                        Check(&classification_Result_1,9);

                    }

                    //以下是辅助窗口的识别
                    Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (Check_Straight(0.5f))
                    {
                        classification_Result_2 = 6;//6直道
                    }
                    else if (is_Slope != 0)
                    {
                        classification_Result_TEMP = 9;
                        Check(&classification_Result_TEMP,9);
                    }
                    else
                    {
                        Classification_Classic36(2,&classification_Result_2,&classification_Result_2_2nd);//多分类算法Classification_25()，传统特征点法Classification_Classic()，模糊道路法Classification_Classic36()
                        Check(&classification_Result_2,classification_Result_2_2nd);
                        Check(&classification_Result_2,9);
                    }


                    Set_Search_Range(0,height_Inverse_Perspective,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    if (Check_Straight(0.7f))
                    {
                        classification_Result_TEMP=6;
                        classification_Result_1=6;
                    }
                    //改回默认窗口
                    Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
                    rt_mutex_release(search_range_mutex);

                    //检查长直道是否满足
                    if((classification_Result_1==6||classification_Result_1==5) && (classification_Result_TEMP==6))
                    {
                        Long_Straight_Flag = 1;//长直道
                    }
                    else
                    {
                        Long_Straight_Flag = 0;//长直道
                    }

                    // 辅助窗口的作用
                    if (classification_Result_2==2||classification_Result_2==3||classification_Result_2==5||classification_Result_2==4||classification_Result_2==12||classification_Result_2==13)
                    {
                        if (classification_Result_TEMP == 7 || classification_Result_TEMP == 8 || classification_Result_TEMP == 9)
                        {
                            classification_Result_TEMP = classification_Result_2;
                            Helper_Window_Flag = 1;
                        }
                        else
                        {
                            Helper_Window_Flag = 0;
                        }
                    }
                }
                classification_Result = classification_Result_TEMP;
            }
            else
            {
                rt_mutex_release(search_range_mutex);
            }

            static uint8 now_signal = 0;
            static uint8 last_signal = 0;
            if (classification_Result == 5)
            {

                now_signal = 2;
                if (now_signal != last_signal)
                {
                    rt_mq_send(message_queue,                           // 消息队列控制块
                        (void *)&now_signal,                             // 发送数据的地址
                        1);                                          // 发送字节数//发信号
                }
                last_signal = now_signal;
            }
            else
            {
                now_signal = 0;
                if (now_signal != last_signal)
                {
                    rt_mq_send(message_queue,                           // 消息队列控制块
                        (void *)&now_signal,                             // 发送数据的地址
                        1);                                      // 发送字节数//发信号
                }
                last_signal = now_signal;
            }
//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_end_Basic:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
            end_thread[4]=rt_tick_get();
//                print_flag=0;
//            }

            rt_event_send(event, EVENT_FLAG8);
        }
    }
}

void CenterLine_entry(void)
{
    rt_uint32_t e;
    while(1)
    {
        if(rt_event_recv
                (event,                                                 // 事件控制块
                        (EVENT_FLAG7),                            // 事件标志3和事件标志5
                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
                        RT_WAITING_FOREVER,                                     // 一直等待
                        &e) == RT_EOK)
        {
//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_start_CenterLine:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
            start_thread[5]=rt_tick_get();
//                print_flag=0;
//            }
            DrawCenterLine();
//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_end_CenterLine:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
//                print_flag=0;
            end_thread[5]=rt_tick_get();
//            }
            rt_event_send(event, EVENT_FLAG9);

        }
    }

}

void Is_Emergency_Stop(void)
{
//    rt_uint32_t e;
//    while(1)
//    {
//        if(rt_event_recv
//                (event,                                                 // 事件控制块
//                        (EVENT_FLAG1),                            // 事件标志3和事件标志5
//                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
//                        RT_WAITING_NO,                                     // 一直等待
//                        &e) == RT_EOK)
//        {
//            if(print_flag==0)
//            {
//                print_flag=1;
//    rt_enter_critical();
//                rt_kprintf("%d_start_Emergency:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
    start_thread[6]=rt_tick_get();
//                print_flag=0;
//            }

            static int cnt = 0;
            if (Real_Volt<7.0f)
            {
                cnt++;
            }
            else
            {
                cnt=0;
            }
            if((Check_TRoad(0,0.1f,3) && zebra_status!=starting && is_Slope == 0)||cnt>5)
            {
                emergency_Stop=1;
            }


            static uint8 now_signal = 0;
            static uint8 last_signal = 0;
            if (emergency_Stop==1)
            {

                now_signal = 4;
                if (now_signal != last_signal)
                {
                    rt_mq_send(message_queue,                           // 消息队列控制块
                        (void *)&now_signal,                             // 发送数据的地址
                        1);                                        // 发送字节数//发信号
                }
                last_signal = now_signal;
            }
            else
            {

                now_signal = 0;
                if (now_signal != last_signal)
                {
                    rt_mq_send(message_queue,                           // 消息队列控制块
                        (void *)&now_signal,                             // 发送数据的地址
                        1);                                   // 发送字节数//发信号
                }
                last_signal = now_signal;
            }
//            if(print_flag==0)
//            {
//                print_flag=1;
//            rt_enter_critical();
//                rt_kprintf("%d_end_Emergency:%d\n", flag_count,rt_tick_get());
//                rt_exit_critical();
            end_thread[6]=rt_tick_get();
//                print_flag=0;
//            }
//        }

//        rt_thread_mdelay(10);
//    }
}


void Confirm_Speed(void)

{
    uint8 set_flag=0;//表示这次是否设置了速度

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


            if(set_flag==0)
            {
                speed_Status = High;
                Reset_Timer(6);
            }



}


void Confirm_Motion(void)
{
    rt_uint32_t e;
    float steeringPID_ratio_kp = 1.0f;
    float steeringPID_ratio_kd = 1.0f;
    float SightForward_ratio = 1.0f;
    float OuterSide_Ratio_ratio = 1.0f;
    float InnerSide_Ratio_ratio = 1.0f;
    float speed_Target_ratio = 1.0f;
//    while(1)
//    {
//        if(rt_event_recv
//                (event,                                                 // 事件控制块
//                        (EVENT_FLAG11),                            // 事件标志3和事件标志5
//                        (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),              // 事件与触发，接收完成后清除事件标志位
//                        RT_WAITING_FOREVER,                                     // 一直等待
//                        &e) == RT_EOK)
//        {
            if(flag_For_T == 2)//T字
            {
                speed_Target_ratio = 0.9f;
            }

            if (flag_For_Left_Circle == 2)
            {
                speed_Target_ratio = 0.95f;
                steeringPID_ratio_kp = 1.8f;//0.85f;
                steeringPID_ratio_kd = 0.6f;
                SightForward_ratio = 0.6f;
                OuterSide_Ratio_ratio = 1.5f;
                InnerSide_Ratio_ratio = 1.1f;
            }

            if (flag_For_Left_Circle == 1)
            {
                speed_Target_ratio = 0.95f;
                steeringPID_ratio_kp = 1.8f;//0.85f;
                steeringPID_ratio_kd = 0.7;
                SightForward_ratio = 0.6f;
                OuterSide_Ratio_ratio = 1.5f;
                InnerSide_Ratio_ratio = 1.5f;
            }


            //右圆环奇怪的偏小

            if (flag_For_Right_Circle == 2)
            {
                speed_Target_ratio = 0.95f;
                steeringPID_ratio_kp = 1.8f;//0.85f;
                steeringPID_ratio_kd = 0.6f;
                SightForward_ratio = 0.6f;
                OuterSide_Ratio_ratio = 1.5f;
                InnerSide_Ratio_ratio = 1.1f;
            }


            if (flag_For_Right_Circle==1)
            {
                speed_Target_ratio = 0.95f;
                steeringPID_ratio_kp = 1.8f;//0.85f;
                steeringPID_ratio_kd = 0.7f;
                SightForward_ratio = 0.6f;
                OuterSide_Ratio_ratio = 1.5f;
                InnerSide_Ratio_ratio = 1.5f;
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
            if (zebra_status == starting)
            {
                OuterSide_Ratio_ratio = 4.0f/2.5f;
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
            Cal_Steering_Target();//由误差（全局变量，待定义）根据位置式PD原理求转向目标Steering_Target(范围-30~30，负数左转，正数右转)

//            rt_event_send(event, EVENT_FLAG13);

//        }
//    }
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

void Motion_Confirm_entry(void)
{
    int count;
    int flagcount_pre=-1;
    while(1)
    {
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_start_MotionConfirm:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();
        //start_thread[7]=rt_tick_get();
//            print_flag=0;
//        }
        if(flagcount_pre==flag_count)
        {
            count++;
        }
        else
        {
            count=0;
            flagcount_pre=flag_count;
        }
        switch(count)
        {
            case 0:start_thread[7]=rt_tick_get();break;
            case 1:start_thread[18]=rt_tick_get();break;
            case 2:start_thread[19]=rt_tick_get();break;
        }


        Confirm_Speed();//确定速度状态
        Confirm_Motion();//根据速度状态进行相关计算
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_end_MotionConfirm:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();
        end_thread[7]=rt_tick_get();
//            print_flag=0;
//        }
        switch(count)
                {
                    case 0:end_thread[7]=rt_tick_get();break;
                    case 1:end_thread[18]=rt_tick_get();break;
                    case 2:end_thread[19]=rt_tick_get();break;
                }
        rt_thread_mdelay(10);
    }
}



void camera_thread_entry(void *parameter)
{
    rt_uint32_t e;
    uint8 counter=0;
    rt_kprintf("camera thread is running.\n");
    //My_Init_Camera();

    while(1)
    {

        if (RT_EOK == rt_sem_take(dma_sem, RT_WAITING_NO))
        {
            InsertTimer2Point(6);
//            rt_kprintf("%d:\n", flag_count);
//            for (counter=0;counter<20;counter++)
//            {
//                rt_kprintf("%d,%d,%d\n", start_thread[counter], end_thread[counter],counter);
//            }
//            flag_count++;
//            if(print_flag==0)
//            {
//                print_flag=1;
            //rt_enter_critical();
            start_thread[0]=rt_tick_get();
             //   rt_exit_critical();
//                print_flag=0;
//            }
            mt9v03x_finish_flag = 0;//表示可以更新mt9v03x_image了

            /*1*/Image_Process();//二值化+逆透视
            rt_event_send(event, EVENT_FLAG2);
            rt_event_send(event, EVENT_FLAG3);
            /*2*///Update_Left_Unknown_or_Right();//求左中右得分
            /*2*///Check_Special_Classification();//特殊识别对应的策略
            rt_event_recv(event,(EVENT_FLAG4 | EVENT_FLAG5), (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),RT_WAITING_FOREVER,&e);
            rt_event_send(event, EVENT_FLAG6);
            rt_event_send(event, EVENT_FLAG7);
            rt_event_recv(event,(EVENT_FLAG8 | EVENT_FLAG9), (RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR),RT_WAITING_FOREVER,&e);
            /*3*///Basic_Classification();//基础识别
            /*4*///DrawCenterLine();//画线
            /*5*///Confirm_Speed();//确定速度状态
            /*5*///Confirm_Motion();//根据速度状态进行相关计算
//            rt_event_send(event, EVENT_FLAG1);
            /*2*/Is_Emergency_Stop();

//            if(print_flag==0)
//            {
//                print_flag=1;
           // rt_enter_critical();
            end_thread[0]=rt_tick_get();
               // rt_kprintf("%d_end_camera:%d\n", flag_count,rt_tick_get());
            //    rt_exit_critical();
//                print_flag=0;
//            }
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
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_start_oled:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();
        start_thread[8]=rt_tick_get();
//            print_flag=0;
//        }
        Update_OLED_per10ms();
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_end_oled:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();
        end_thread[8]=rt_tick_get();
//            print_flag=0;
//        }
        rt_thread_mdelay(20);
    }
}


void ZEBRA_thread_entry(void *parameter)
{
    rt_kprintf("ZEBRA thread is running.\n");
    static int count;
    static int flagcount_pre=-1;
    while(1)
    {
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_start_zebra:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();

//            print_flag=0;
//        }
        if(flagcount_pre==flag_count)
        {
            count++;
        }
        else
        {
            count=0;
            flagcount_pre=flag_count;
        }
        switch(count)
        {
            case 0:start_thread[9]=rt_tick_get();break;
            case 1:start_thread[14]=rt_tick_get();break;
            case 2:start_thread[15]=rt_tick_get();break;
        }
        //检查斑马线数据
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
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_end_zebra:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();

        static uint8 now_signal = 0;
        static uint8 last_signal = 0;
        if (zebra_status == banning||zebra_status == finishing)
        {

            now_signal = 1;
            if (now_signal != last_signal)
            {
                rt_mq_send(message_queue,                           // 消息队列控制块
                    (void *)&now_signal,                             // 发送数据的地址
                    1);                                        // 发送字节数//发信号
            }
            last_signal = now_signal;
        }
        else
        {

            now_signal = 0;
            if (now_signal != last_signal)
            {
                rt_mq_send(message_queue,                           // 消息队列控制块
                    (void *)&now_signal,                             // 发送数据的地址
                    1);                                   // 发送字节数//发信号
            }
            last_signal = now_signal;
        }


        switch(count)
        {
            case 0:end_thread[9]=rt_tick_get();break;
            case 1:end_thread[14]=rt_tick_get();break;
            case 2:end_thread[15]=rt_tick_get();break;
        }
//            print_flag=0;
//        }

        rt_thread_mdelay(10);

    }
}


void VL53L0X_thread_entry(void *parameter)
{
    rt_kprintf("VL53L0X thread is running.\n");
//    if (flag_for_ICM_Init == 0)
//    {
//        VL53L0X_Init();
//        //            My_Init_ICM();//我的初始化ICM
//        //            Get_Zero_Bias();//求陀螺仪零漂值
//        flag_for_ICM_Init = 1;
//
//    }
    while(1)
    {
//        if(print_flag==0)
//                {
//                    print_flag=1;
//        rt_enter_critical();
//                    rt_kprintf("%d_start_laser:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
        start_thread[10]=rt_tick_get();
//                    print_flag=0;
//                }
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

        static uint8 now_signal = 0;
        static uint8 last_signal = 0;
        if (is_Slope==1||is_Slope==2||is_Slope==3)
        {

            now_signal = 3;
            if (now_signal != last_signal)
            {
                rt_mq_send(message_queue,                           // 消息队列控制块
                    (void *)&now_signal,                             // 发送数据的地址
                    1);                                        // 发送字节数//发信号
            }
            last_signal = now_signal;
        }
        else
        {

            now_signal = 0;
            if (now_signal != last_signal)
            {
                rt_mq_send(message_queue,                           // 消息队列控制块
                    (void *)&now_signal,                             // 发送数据的地址
                    1);                                   // 发送字节数//发信号
            }
            last_signal = now_signal;
        }


//        if(print_flag==0)
//                {
//                    print_flag=1;
//        rt_enter_critical();
//                    rt_kprintf("%d_end_laser:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
        end_thread[10]=rt_tick_get();
//                    print_flag=0;
//                }
        rt_thread_mdelay(10);
    }
}


void ADC_thread_entry(void *parameter)
{

//    rt_kprintf("ADC thread is running.\n");
    //My_Init_ADC();
    int count;
    int flagcount_pre=-1;
    while(1)
    {
//        if(print_flag==0)
//        {
//            print_flag=1;
//        rt_enter_critical();
//            rt_kprintf("%d_start_adc:%d\n", flag_count,rt_tick_get());
//            rt_exit_critical();
//        start_thread[11]=rt_tick_get();
//            print_flag=0;
//        }
        if(flagcount_pre==flag_count)
        {
            count++;
        }
        else
        {
            count=0;
            flagcount_pre=flag_count;
        }
        switch(count)
        {
            case 0:start_thread[11]=rt_tick_get();break;
            case 1:start_thread[16]=rt_tick_get();break;
            case 2:start_thread[17]=rt_tick_get();break;
        }

        Get_ADC_DATA();
//        if(print_flag==0)
//                {
//                    print_flag=1;
//        rt_enter_critical();
//                    rt_kprintf("%d_end_adc:%d\n", flag_count,rt_tick_get());
//                    rt_exit_critical();
//        end_thread[11]=rt_tick_get();
//                    print_flag=0;
//                }
        switch(count)
                {
                    case 0:end_thread[11]=rt_tick_get();break;
                    case 1:end_thread[16]=rt_tick_get();break;
                    case 2:end_thread[17]=rt_tick_get();break;
                }
        rt_thread_mdelay(10);
    }
}

void LED_thread_entry(void *parameter)
{
    //rt_kprintf("LED thread is running.\n");
    //My_Init_LED();
    while(1)
    {
        //gpio_toggle(P10_5);
        //gpio_toggle(P10_6);
        rt_thread_mdelay(500);
    }
}

void Warning_LED_thread_entry(void *parameter)
{
    //rt_kprintf("Warrning_LED thread is running.\n");
    //My_Init_LED();
    static uint8 recv_test;
    while(1)
    {
        rt_mq_recv(message_queue,                           // 消息队列控制块
                    (void *)&recv_test,                             // 接收数据的地址
                    1,                                              // 接收数量
                    RT_WAITING_FOREVER);                            // 一直等待有数据采退出接收函数
        switch(recv_test)
        {
            case 0://不亮
                gpio_set(P10_6,1);gpio_set(P10_5,1);
                break;
            case 1://yellow
                gpio_set(P10_6,0);gpio_set(P10_5,1);
                break;
            case 2://red
                gpio_set(P10_6,1);gpio_set(P10_5,0);
                break;
            case 3://yellow&red
                gpio_set(P10_6,0);gpio_set(P10_5,0);
                break;
            case 4://0.5s亮，0.5s灭
                while(1)
                {
                    gpio_toggle(P10_6);gpio_toggle(P10_5);rt_thread_mdelay(500);
                }
                break;

        }
        rt_thread_mdelay(50);
    }
}

void KS_timer_entry(void *parameter)
{
        //Update_OLED_per10ms();
//    if(print_flag==0)
//           {
//               print_flag=1;
//               rt_kprintf("%d_start_ks:%d\n", flag_count,rt_tick_get());
//               print_flag=0;
//           }
    start_thread[12]=rt_tick_get();
        Check_Key_per10ms();
        Check_Switch_per10ms();
//        if(print_flag==0)
//               {
//                   print_flag=1;
//                   rt_kprintf("%d_end_ks:%d\n", flag_count,rt_tick_get());
//                   print_flag=0;
//               }
        end_thread[12]=rt_tick_get();
}

void motion_timer_entry(void *parameter)
{
//    if(print_flag==0)
//           {
//               print_flag=1;
//               rt_kprintf("%d_start_motion:%d\n", flag_count,rt_tick_get());
//               print_flag=0;
//           }
    start_thread[13]=rt_tick_get();
    if (emergency_Stop == 0)
    {
        //由速度、转向角度的目标值，通过PID等算法，改变直流电机和舵机的状态
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
        Cal_Speed_Output1();//新增
        Cal_Speed_Output2();//新增
        Set_Speed1();
        Set_Speed2();
        Set_Steering();
    }
//    if(print_flag==0)
//           {
//               print_flag=1;
//               rt_kprintf("%d_end_motion:%d\n", flag_count,rt_tick_get());
//               print_flag=0;
//           }
    end_thread[13]=rt_tick_get();
}






int TIMETIME_timer_create (void)
{
    rt_timer_t timer1;
    // 创建一个定时器 周期100个tick
    timer1 = rt_timer_create(
        "timer_ms",                                           // timer1表示定时器的名称，8个字符内。
        Timer_Action_per1ms,                                          // timerout1表示时间到了之后需要执行的函数
        RT_NULL,                                            // RT_NULL表示不需要传递参数。
        1,                                                // 100表示定时器的超时时间为100个系统tick，系统周期为1毫秒，则100表示100毫秒
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC表示定时器以周期运行  如果设置为RT_TIMER_FLAG_ONE_SHOT则只会运行一次

    // 首先检查定时器控制块不是空，则启动定时器
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }

    return 0;
}

int KS_timer_create (void)
{
    rt_timer_t timer1;
    // 创建一个定时器 周期100个tick
    timer1 = rt_timer_create(
        "timer_KS",                                           // timer1表示定时器的名称，8个字符内。
        KS_timer_entry,                                          // timerout1表示时间到了之后需要执行的函数
        RT_NULL,                                            // RT_NULL表示不需要传递参数。
        10,                                                // 100表示定时器的超时时间为100个系统tick，系统周期为1毫秒，则100表示100毫秒
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC表示定时器以周期运行  如果设置为RT_TIMER_FLAG_ONE_SHOT则只会运行一次

    // 首先检查定时器控制块不是空，则启动定时器
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }

    return 0;
}

int motion_timer_create(void)
{
    rt_timer_t timer1;
    // 创建一个定时器 周期100个tick
    timer1 = rt_timer_create(
        "timer_mot",                                           // timer1表示定时器的名称，8个字符内。
        motion_timer_entry,                                          // timerout1表示时间到了之后需要执行的函数
        RT_NULL,                                            // RT_NULL表示不需要传递参数。
        10,                                                // 100表示定时器的超时时间为100个系统tick，系统周期为1毫秒，则100表示100毫秒
        RT_TIMER_FLAG_PERIODIC);                            // RT_TIMER_FLAG_PERIODIC表示定时器以周期运行  如果设置为RT_TIMER_FLAG_ONE_SHOT则只会运行一次
    // 首先检查定时器控制块不是空，则启动定时器
    if(timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    return 0;
}

int LED_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("LED",                      // 线程名称
            LED_thread_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        512,                                                // 512 个字节的栈空间
        20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        5);                                                 // 时间片为5

//    rt_kprintf("create LED thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("LED thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("LED thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int Warning_LED_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("Warning_LED",                      // 线程名称
            Warning_LED_thread_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        512,                                                // 512 个字节的栈空间
        20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        5);                                                 // 时间片为5

//    rt_kprintf("create Warning_LED thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("Warning_LED thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("Warning_LED thread create ERROR.\n");
        return 1;
    }
    return 0;
}


int ADC_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("ADC",                      // 线程名称
            ADC_thread_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        512,                                                // 512 个字节的栈空间
        20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        5);                                                 // 时间片为5

//    rt_kprintf("create ADC thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("ADC thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("ADC thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int VL53L0X_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("VL53L0X",                      // 线程名称
            VL53L0X_thread_entry,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create VL53L0X thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("VL53L0X thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("VL53L0X thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int ZEBRA_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("ZEBRA",                      // 线程名称
            ZEBRA_thread_entry,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create ZEBRA thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("ZEBRA thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("ZEBRA thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int OLED_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("OLED",                      // 线程名称
            OLED_thread_entry,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create OLED thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("OLED thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("OLED thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int camera_thread_create(void)
{
    mt9v03x_finish_sem = rt_sem_create("mt9v03x_finish_sem", 0 ,RT_IPC_FLAG_FIFO);
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("camera",                      // 线程名称
            camera_thread_entry,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create camera thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("camera thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("camera thread create ERROR.\n");
        return 1;
    }

    return 0;
}




int CenterLine_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("CenterLine",                      // 线程名称
            CenterLine_entry,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create CenterLine thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("CenterLine thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("CenterLine thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int Basic_Classification_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("Basic_Classification",                      // 线程名称
            Basic_Classification,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create Basic_Classification thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("Basic_Classification thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("Basic_Classification thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int Check_Special_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("Check_Special",                      // 线程名称
            Check_Special_Classification,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create Check_Special thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("Check_Special thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("Check_Special thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int Left_or_Right_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("Left_or_Right",                      // 线程名称
            Update_Left_Unknown_or_Right,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create Left_or_Right thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("Left_or_Right thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("Left_or_Right thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int Emergency_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("Emergency",                      // 线程名称
            Is_Emergency_Stop,                                      // 线程入口函数
            RT_NULL,                                            // 线程参数
            512,                                                // 512 个字节的栈空间
            20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
            5);                                                 // 时间片为5

//    rt_kprintf("create Emergency thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("Emergency thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("Emergency thread create ERROR.\n");
        return 1;
    }

    return 0;
}

int MotionConfirm_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("MotionConfirm",                      // 线程名称
            Motion_Confirm_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        512,                                                // 512 个字节的栈空间
        20,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        5);                                                 // 时间片为5

//    rt_kprintf("create MotionConfirm thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
//        rt_kprintf("MotionConfirm thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
    {
        rt_kprintf("MotionConfirm thread create ERROR.\n");
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
    event = rt_event_create("event", RT_IPC_FLAG_FIFO);
    message_queue = rt_mq_create("mq1",1,10,RT_IPC_FLAG_FIFO);
    search_range_mutex = rt_mutex_create("search_range_mutex", RT_IPC_FLAG_FIFO);

    My_Init_Key();
    oled_init();
    My_Init_Switch();
    My_Init_Camera();
    My_Init_SpeedSensor1();//我的初始化编码器
    My_Init_SpeedSensor2();//我的初始化编码器
    My_Init_Steering();//我的初始化舵机
    My_Init_Motor1();//我的初始化直流电机
    My_Init_Motor2();//我的初始化直流电机
    My_Init_ADC();
    VL53L0X_Init();



    OLED_thread_create();
    //KEY_thread_create();
    //SWITCH_thread_create();
    KS_timer_create();
    camera_thread_create();
    Left_or_Right_thread_create();
    Check_Special_thread_create();
    Basic_Classification_thread_create();
    CenterLine_thread_create();
    MotionConfirm_thread_create();
    Warning_LED_thread_create();
    //Emergency_thread_create();
    //motion_thread_create();
    motion_timer_create();
    ZEBRA_thread_create();
    ADC_thread_create();
    VL53L0X_thread_create();

    Get_Inverse_Perspective_Table();//求逆透视表
}

int init_thread_create(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("init",                      // 线程名称
            init_thread_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        512,                                                // 512 个字节的栈空间
        5,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        5);                                                 // 时间片为5

    rt_kprintf("create init thread.\n");
    if(tid != RT_NULL)                                     // 线程创建成功
    {
        rt_kprintf("init thread create OK.\n");
        rt_thread_startup(tid);                            // 运行该线程
    }
    else                                                    // 线程创建失败
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
    //    get_clk();//获取时钟频率  务必保留
    //用户在此处调用各种初始化函数等
    //    EEPROM_Write_Data(ID_STEERING_DUTY_CENTER, &STEERING_DUTY_CENTER);//先写进去
    //  STEERING_DUTY_CENTER = EEPROM_Read_Data(ID_STEERING_DUTY_CENTER,uint32);//再读取
//    My_Init_Steering();//我的初始化舵机
//    My_Init_Motor1();//我的初始化直流电机
//    My_Init_Motor2();//我的初始化直流电机
//    My_Init_SpeedSensor1();//我的初始化编码器
//    My_Init_SpeedSensor2();//我的初始化编码器
//    My_Init_OLED();//我的初始化OLED
//    My_Init_Camera();//我的初始化摄像头
//    My_Init_Switch();//我的初始化拨盘
//    My_Init_Key();//我的初始化按键
//    My_Init_UART();//我的初始化串口通信
//    My_Init_FuzzyPID_Speed();//我的初始化速度模糊PID控制
//    My_Init_Timer();//我的初始化TIMER

//    My_Init_ADC();//我的初始化ADC
//    if (flag_for_ICM_Init == 0)
//    {
//        VL53L0X_Init();
//        //            My_Init_ICM();//我的初始化ICM
//        //            Get_Zero_Bias();//求陀螺仪零漂值
//        flag_for_ICM_Init = 1;
//
//    }






    //等待所有核心初始化完毕
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);






    //初始化LED引脚

    while(1)
    {
        //翻转LED引脚


        rt_thread_mdelay(500);
    }
}

#pragma section all restore


