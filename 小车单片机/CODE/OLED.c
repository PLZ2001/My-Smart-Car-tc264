#include "headfile.h"
#include "OLED.h"
#include "CAMERA.h"
#include "UART.h"
#include "STEERING.h"
#include "fuzzy_PID.h"
#include "TIME.h"
#include "MOTOR1.h"
#include "MOTOR2.h"


enum OLEDPage OLED_Page = Timer_Page;
uint8 OLED_EN = TRUE;//用于表示OLED屏幕是否开启
uint8 OLED_Page_Active_Flag = TRUE;//用于表示OLED屏幕是否切换页面

void My_Init_OLED(void)
{
    oled_init();
    pit_interrupt_ms(CCU6_0, PIT_CH1, 16);//Update_OLED_per16ms
}

void Update_OLED_per16ms(void)
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
               if (Read_Timer_Status() == PAUSED)
               {
                   OLED_PRINTF(0,0,"Timer Status: PAUSED ");
               }
               else
               {
                   OLED_PRINTF(0,0,"Timer Status: RUNNING");
               }
               OLED_PRINTF(0,1,"Time Now: %.3f s   ",Read_Timer());
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
