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
#include "SEARCH.h"


enum OLEDPage OLED_Page = TimeSet_Page;
uint8 OLED_EN = TRUE;//���ڱ�ʾOLED��Ļ�Ƿ���
uint8 OLED_Page_Active_Flag = TRUE;//���ڱ�ʾOLED��Ļ�Ƿ��л�ҳ��

uint8 pointer_temp = 0;//������Ϊָ��ʹ��
int8 up_Down = 1;//��TImeSet_Pageʹ�ã�1��ʾ���ӣ�-1��ʾ����

void My_Init_OLED(void)
{
    oled_init();
    pit_interrupt_ms(CCU6_0, PIT_CH1, 10);//Update_OLED_per10ms
}

void Update_OLED_per10ms(void)
{
    if (OLED_EN == TRUE)
    {
        //����OLEDӦ����ʾ�����ݣ�ˢ��OLED��Ļ
        if (OLED_Page_Active_Flag == TRUE)
        {
            oled_fill(0x00);
            OLED_Page_Active_Flag = FALSE;
        }//ÿ���а���������ˢ��һ��
        switch(OLED_Page)
        {
           case Camera_Page:
               oled_dis_bmp(height_Inverse_Perspective, width_Inverse_Perspective, *mt9v03x_image_cutted_thresholding_inversePerspective, 0);
               break;
           case UART_Debug_Page:
               //��ʾ������Cpu0_Main.c�Ĵ���ͨ�Ų������
               break;
           case TimeSet_Page:
               if (pointer_temp == 0)
               {
                   OLED_PRINTF(0,0,"->3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
               }
               else if (pointer_temp == 1)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"->RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
               }
               else if (pointer_temp == 2)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"->RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
               }
               else if (pointer_temp == 3)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"->RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
               }
               if (up_Down == 1)
               {
                   OLED_PRINTF(0,4,"ADD");
               }
               else if (up_Down == -1)
               {
                   OLED_PRINTF(0,4,"SUB");
               }
               break;
           case Speed_Page:
               OLED_PRINTF(0,0,"Speed1:%01.05fm/s   ",speed_Measured1);
               OLED_PRINTF(0,1,"Speed2:%01.05fm/s   ",speed_Measured2);
               OLED_PRINTF(0,2,"Steering:%02.04f   ",steering_Target);
               OLED_PRINTF(0,3,"SpeedTarget:%01.01fm/s   ",speed_Target);
               OLED_PRINTF(0,4,"Class:%d     ",classification_Result);
               OLED_PRINTF(0,5,"2-value:%d     ",thresholding_Value);
               if (start_Flag == 1)
                {
                   OLED_PRINTF(0,6,"START: ON ");
                }
                else
                {
                    OLED_PRINTF(0,6,"START: OFF");
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
               OLED_PRINTF(0,1,"Timer4 Now: %.3f s   ",Read_Timer(4));
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
              //��ʾ������Cpu1_Main.c�������
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
