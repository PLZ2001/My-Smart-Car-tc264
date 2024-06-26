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
#include "BEEP.h"
//#include "EEPROM.h"


#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。
int core0_main(void)
{
    disableInterrupts();
	get_clk();//获取时钟频率  务必保留
	//用户在此处调用各种初始化函数等
//    EEPROM_Write_Data(ID_STEERING_DUTY_CENTER, &STEERING_DUTY_CENTER);//先写进去
//	STEERING_DUTY_CENTER = EEPROM_Read_Data(ID_STEERING_DUTY_CENTER,uint32);//再读取


	My_Init_Steering();//我的初始化舵机
	My_Init_Motor1();//我的初始化直流电机
	My_Init_Motor2();//我的初始化直流电机
	My_Init_SpeedSensor1();//我的初始化编码器
	My_Init_SpeedSensor2();//我的初始化编码器
    My_Init_OLED();//我的初始化OLED
    My_Init_Camera();//我的初始化摄像头
    My_Init_Switch();//我的初始化拨盘
    My_Init_Key();//我的初始化按键
//    My_Init_UART();//我的初始化串口通信
    My_Init_FuzzyPID_Speed();//我的初始化速度模糊PID控制
    My_Init_Timer();//我的初始化TIMER

    My_Init_ADC();//我的初始化ADC



    My_Init_LED();
    My_Init_BEEP();
    //My_Init_Wifi();//我的初始化WIFI通信

    //等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();

    //注意 从V1.1.6版本之后  printf打印的信息从串口输出具体可以学习库例程6-Printf_Demo
	while (TRUE)
	{
        if (flag_for_ICM_Init == 0)
        {
            VL53L0X_Init();
//            My_Init_ICM();//我的初始化ICM
//            Get_Zero_Bias();//求陀螺仪零漂值
            flag_for_ICM_Init = 1;
        }

        Get_ADC_DATA();//更新电压读取


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

//        Get_ICM_DATA();//更新陀螺仪数据
//        Check_Slope_with_Gyro();

//	    if (UART_EN == TRUE)
//	    {
//	        UART(Send);
//	    }
//	    else
//	    {
//	        UART(Emergency_Send);
//	    }
	    //Send_with_Wifi();//用wifi发送

	}
}



#pragma section all restore


