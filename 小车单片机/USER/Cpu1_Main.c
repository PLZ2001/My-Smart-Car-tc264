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
#include "MOTOR.h"//直流电机相关
#include "OLED.h"//显示屏相关
#include "STEERING.h"//舵机相关
#include "UART.h"
#include "fastlz.h"
#include "TIME.h"


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
            Get_Cutted_Image();//裁剪图像到188*40
            mt9v03x_finish_flag = 0;//表示可以更新mt9v03x_image了

            Get_Thresholding_Image();
            Get_Inverse_Perspective_Image();

            static uint8 flag_For_Right_Circle = 0;
            //如果是3右环岛中心，且定时器没有在计时，就开定时
            if (Read_Timer_Status() == PAUSED && classification_Result == 3)
            {
                if (flag_For_Right_Circle == 0)//说明还没进右环岛
                {
                    flag_For_Right_Circle = 1;
                    classification_Result = 14;//靠右
                }
                else //说明准备出右环岛
                {
                    flag_For_Right_Circle = 0;
                    classification_Result = 13;//靠左
                }
                Start_Timer();
            }
            //如果在计时，判断计时是否达到要求时间
            if (Read_Timer_Status() == RUNNING)
            {
                switch (classification_Result)
                {
                    case 13:
                        if (Read_Timer()>1.0f) //13靠左是计时1s
                        {
                            Reset_Timer();
                        }
                        break;
                    case 14:
                        if (Read_Timer()>3.0f) //14靠右是计时3s
                        {
                            Reset_Timer();
                        }
                        break;
                    default:
                        break;
                }
            }
            //如果不在计时，继续分类
            if (Read_Timer_Status() == PAUSED)
            {
                //小车处于右圆环状态
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
                    //只有当再次识别到右圆环中心时，才可以flag_For_Right_Circle=0，从而进行正常的识别，否则一直靠右行驶
                    if (classification_Result != 3)
                    {
                        classification_Result = 14;
                    }
                }
                else
                {//正常识别
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

            //由处理后的图像等信息，获取速度、转向角度的目标值
            Cal_Steering_Error(0.5);//根据Col_Center和扫描范围search_Lines计算误差（全局变量，待定义）
            Cal_Speed_Target();//根据Col_Center和扫描范围search_Lines计算速度目标speed_Target，待完成

            if (UART_EN == TRUE)
            {
                UART_Flag_TX = TRUE;
            }
        }



        //低速目标且低速时，开环
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
