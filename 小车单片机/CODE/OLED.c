#include "headfile.h"
#include "OLED.h"
#include "CAMERA.h"
#include "UART.h"
#include "STEERING.h"
#include "fuzzy_PID.h"
#include "TIME.h"
#include "MOTOR1.h"
#include "MOTOR2.h"
#include "MOTOR_CTL.h"


enum OLEDPage OLED_Page = Timer_Page;
uint8 OLED_EN = TRUE;//用于表示OLED屏幕是否开启
uint8 OLED_Page_Active_Flag = TRUE;//用于表示OLED屏幕是否切换页面

void My_Init_OLED(void)
{
    oled_init();
    pit_interrupt_ms(CCU6_0, PIT_CH1, 10);//Update_OLED_per10ms
}

void Update_OLED_per10ms(void)
{
    if (OLED_EN == TRUE)
    {
        //根据OLED应该显示的内容，刷新OLED屏幕
        if (OLED_Page_Active_Flag == TRUE)
        {
            oled_fill(0x00);
            OLED_Page_Active_Flag = FALSE;
        }//每次有按键动作就刷屏一下
        switch(OLED_Page)
        {
           case Camera_Page:
               oled_dis_bmp(64, 128, *mt9v03x_image_cutted, thresholding_Value);
               break;
           case UART_Debug_Page:
               //显示内容由Cpu0_Main.c的串口通信部分完成
               break;
           case Speed_Page:
               OLED_PRINTF(0,0,"Speed1:%01.05fm/s   ",speed_Measured1);
               OLED_PRINTF(0,1,"Speed2:%01.05fm/s   ",speed_Measured2);
               OLED_PRINTF(0,2,"Steering:%02.04f   ",steering_Target);
               OLED_PRINTF(0,3,"SpeedTarget:%01.01fm/s   ",speed_Target);
               OLED_PRINTF(0,4,"Class:%d     ",classification_Result);
               if (start_Flag == 1)
                {
                   OLED_PRINTF(0,5,"START: ON ");
                }
                else
                {
                    OLED_PRINTF(0,5,"START: OFF");
                }
               //OLED_PRINTF(0,2,"fuzzy_struct->output[2]:%f   ",pid_vector[0]->fuzzy_struct->output[2]);
               break;
           case UART_Setting_Page:
               if (UART_EN == TRUE)
               {
                   OLED_PRINTF(0,0,"UART: ON ");
               }
               else
               {
                   OLED_PRINTF(0,0,"UART: OFF");
               }
               break;
           case OLED_Setting_Page:
               if (OLED_EN == TRUE)
               {
                   OLED_PRINTF(0,0,"OLED: ON ");
               }
               else
               {
                   OLED_PRINTF(0,0,"OLED: OFF");
               }
               break;
           case Timer_Page:
               if (Read_Timer_Status(0) == PAUSED)
               {
                   OLED_PRINTF(0,0,"Timer0 Status: PAUSED ");
               }
               else
               {
                   OLED_PRINTF(0,0,"Timer0 Status: RUNNING");
               }
               OLED_PRINTF(0,1,"Timer0 Now: %.3f s   ",Read_Timer(0));
               break;
           case Gyroscope_Page:
               OLED_PRINTF(0,0,"gyro_x: %d     ",icm_gyro_x);
               OLED_PRINTF(0,1,"gyro_y: %d     ",icm_gyro_y);
               OLED_PRINTF(0,2,"gyro_z: %d     ",icm_gyro_z);
               OLED_PRINTF(0,3,"acc_x: %d     ",icm_acc_x);
               OLED_PRINTF(0,4,"acc_y: %d     ",icm_acc_y);
               OLED_PRINTF(0,5,"acc_z: %d     ",icm_acc_z);
               break;
           case Timer_Page2:
              //显示内容由Cpu1_Main.c部分完成
              break;
           default:
               oled_fill(0x00);
               break;
        }
    }
    else
    {
        oled_fill(0x00);
    }
}
