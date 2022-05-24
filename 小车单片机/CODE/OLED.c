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


enum OLEDPage OLED_Page = Camera_Page2;//SteeringPID_Page;//Speed_Page;//Circle_Page;//Steering_Center_Page;//Camera_Page;//TimeSet_Page;
uint8 OLED_EN = TRUE;//用于表示OLED屏幕是否开启
uint8 OLED_Page_Active_Flag = TRUE;//用于表示OLED屏幕是否切换页面

uint8 pointer_temp = 0;//用来作为指针使用
int8 up_Down = 1;//给TImeSet_Page使用，1表示增加，-1表示减少

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
           case ThreeDot_Page:
               OLED_PRINTF(0,0,"NO1:(%d,%d) ",first_Dot[0],first_Dot[1]);
               OLED_PRINTF(0,1,"NO2:(%d,%d) ",second_Dot[0],second_Dot[1]);
               OLED_PRINTF(0,2,"NO3:(%d,%d) ",third_Dot[0],third_Dot[1]);
               OLED_PRINTF(0,3,"angle:%03.01f ",arccosValue);
               break;
           case Differential_Page:
               if (pointer_temp == 0)
               {
                   OLED_PRINTF(0,0,"->DR:%01.02f ",Differential_Ratio);
                   OLED_PRINTF(0,1,"InnerSide:%01.02f ",InnerSide_Ratio);
                   OLED_PRINTF(0,2,"SightForward:%01.02f ",SightForward);
               }
               else if (pointer_temp == 1)
               {
                   OLED_PRINTF(0,0,"DR:%01.02f ",Differential_Ratio);
                   OLED_PRINTF(0,1,"->InnerSide:%01.02f ",InnerSide_Ratio);
                   OLED_PRINTF(0,2,"SightForward:%01.02f ",SightForward);
               }
               else if (pointer_temp == 2)
               {
                   OLED_PRINTF(0,0,"DR:%01.02f ",Differential_Ratio);
                   OLED_PRINTF(0,1,"InnerSide:%01.02f ",InnerSide_Ratio);
                   OLED_PRINTF(0,2,"->SightForward:%01.02f ",SightForward);
               }
               if (up_Down == 1)
               {
                   OLED_PRINTF(0,3,"ADD");
               }
               else if (up_Down == -1)
               {
                   OLED_PRINTF(0,3,"SUB");
               }
               break;
           case Camera_Page2:
               my_oled_dis_bmp();
               OLED_PRINTF(0,6,"SE:%03.01f S:%02.02f",steering_Error,steering_Target);
//               OLED_PRINTF(0,7,"cnt:%d",cnt);
//               OLED_PRINTF(0,7,"CC10:%03.01f CC20:%03.01f",Col_Center[10],Col_Center[20]);
               OLED_PRINTF(0,7,"2-value:%d Class:%d ",thresholding_Value,classification_Result);
               break;
           case MotorPID_Page:
               if (pointer_temp == 0)
               {
                   OLED_PRINTF(0,0,"->Kp:%01.02f s   ",PID_KP1);
                   OLED_PRINTF(0,1,"Ki:%01.02f s   ",PID_KI1);
                   OLED_PRINTF(0,2,"Kd:%01.02f s   ",PID_KD1);
               }
               else if (pointer_temp == 1)
               {
                   OLED_PRINTF(0,0,"Kp:%01.02f s   ",PID_KP1);
                   OLED_PRINTF(0,1,"->Ki:%01.02f s   ",PID_KI1);
                   OLED_PRINTF(0,2,"Kd:%01.02f s   ",PID_KD1);
               }
               else if (pointer_temp == 2)
               {
                   OLED_PRINTF(0,0,"Kp:%01.02f s   ",PID_KP1);
                   OLED_PRINTF(0,1,"Ki:%01.02f s   ",PID_KI1);
                   OLED_PRINTF(0,2,"->Kd:%01.02f s   ",PID_KD1);
               }
               if (up_Down == 1)
               {
                   OLED_PRINTF(0,3,"ADD");
               }
               else if (up_Down == -1)
               {
                   OLED_PRINTF(0,3,"SUB");
               }
               break;

           case Circle_Page:
               OLED_PRINTF(0,0,"%d:%d   ",Circle_lines,last_angle_up);
               OLED_PRINTF(0,1,"1:%d    ",last_angle_down);
               break;
           case Steering_Center_Page:
               if (pointer_temp == 0)
              {
                  OLED_PRINTF(0,0,"Center:%d    ",STEERING_DUTY_CENTER);
              }
               if (up_Down == 1)
               {
                   OLED_PRINTF(0,1,"ADD");
               }
               else if (up_Down == -1)
               {
                   OLED_PRINTF(0,1,"SUB");
               }
               break;
           case Camera_Page:
               //oled_dis_bmp(height_Inverse_Perspective, width_Inverse_Perspective, *mt9v03x_image_cutted_thresholding_inversePerspective, 0);
               OLED_PRINTF(0,0,"%d %d %d %d %d %d",fuzzy_Image_36[0][0],fuzzy_Image_36[0][1],fuzzy_Image_36[0][2],fuzzy_Image_36[0][3],fuzzy_Image_36[0][4],fuzzy_Image_36[0][5]);
               OLED_PRINTF(0,1,"%d %d %d %d %d %d",fuzzy_Image_36[1][0],fuzzy_Image_36[1][1],fuzzy_Image_36[1][2],fuzzy_Image_36[1][3],fuzzy_Image_36[1][4],fuzzy_Image_36[1][5]);
               OLED_PRINTF(0,2,"%d %d %d %d %d %d",fuzzy_Image_36[2][0],fuzzy_Image_36[2][1],fuzzy_Image_36[2][2],fuzzy_Image_36[2][3],fuzzy_Image_36[2][4],fuzzy_Image_36[2][5]);
               OLED_PRINTF(0,3,"%d %d %d %d %d %d",fuzzy_Image_36[3][0],fuzzy_Image_36[3][1],fuzzy_Image_36[3][2],fuzzy_Image_36[3][3],fuzzy_Image_36[3][4],fuzzy_Image_36[3][5]);
               OLED_PRINTF(0,4,"%d %d %d %d %d %d",fuzzy_Image_36[4][0],fuzzy_Image_36[4][1],fuzzy_Image_36[4][2],fuzzy_Image_36[4][3],fuzzy_Image_36[4][4],fuzzy_Image_36[4][5]);
               OLED_PRINTF(0,5,"%d %d %d %d %d %d",fuzzy_Image_36[5][0],fuzzy_Image_36[5][1],fuzzy_Image_36[5][2],fuzzy_Image_36[5][3],fuzzy_Image_36[5][4],fuzzy_Image_36[5][5]);
               //OLED_PRINTF(0,6,"Class:%d Value:%01.02f",classification_Result,fuzzy_thresholdingValue_36);
               OLED_PRINTF(0,6,"Class:%d Max:%01.02f",classification_Result,max_Score);
               OLED_PRINTF(0,7,"%02.01f %02.01f %02.01f %02.01f",score[0],score[1],score[2],score[3]);
               break;
           case UART_Debug_Page:
               //显示内容由Cpu0_Main.c的串口通信部分完成
               break;
           case SteeringPID_Page:
               OLED_temp();
               break;
           case TimeSet_Page:
               if (pointer_temp == 0)
               {
                   OLED_PRINTF(0,0,"->3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
                   OLED_PRINTF(0,4,"T:%01.02f s   ",T_Time);
               }
               else if (pointer_temp == 1)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"->RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
                   OLED_PRINTF(0,4,"T:%01.02f s   ",T_Time);
               }
               else if (pointer_temp == 2)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"->RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
                   OLED_PRINTF(0,4,"T:%01.02f s   ",T_Time);
               }
               else if (pointer_temp == 3)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"->RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
                   OLED_PRINTF(0,4,"T:%01.02f s   ",T_Time);
               }
               else if (pointer_temp == 4)
               {
                   OLED_PRINTF(0,0,"3Road_R:%01.02f s   ",threeRoads_RightTime);
                   OLED_PRINTF(0,1,"RCircle_R:%01.02f s   ",rightCircle_RightTime);
                   OLED_PRINTF(0,2,"RCircle_L:%01.02f s   ",rightCircle_LeftTime);
                   OLED_PRINTF(0,3,"RCircle_Ban:%01.02f s   ",rightCircle_BannedTime);
                   OLED_PRINTF(0,4,"->T:%01.02f s   ",T_Time);
               }
               if (up_Down == 1)
               {
                   OLED_PRINTF(0,5,"ADD");
               }
               else if (up_Down == -1)
               {
                   OLED_PRINTF(0,5,"SUB");
               }
               break;
           case Speed_Page:
               OLED_PRINTF(0,0,"Speed1:%01.05fm/s   ",speed_Measured1);
               OLED_PRINTF(0,1,"Speed2:%01.05fm/s   ",speed_Measured2);
               OLED_PRINTF(0,2,"Steering:%02.04f   ",steering_Target);
               OLED_PRINTF(0,3,"SpeedTargetMax:%01.01fm/s   ",speed_Target_Max*60.0f/68.0f);
               OLED_PRINTF(0,4,"SpeedTargetMin:%01.01fm/s   ",speed_Target_Min*60.0f/68.0f);
               OLED_PRINTF(0,5,"Class:%d     ",classification_Result);
               OLED_PRINTF(0,6,"2-value:%d     ",thresholding_Value);
               if (start_Flag == 1)
                {
                   OLED_PRINTF(0,7,"START: ON ");
                }
                else
                {
                    OLED_PRINTF(0,7,"START: OFF");
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

void my_oled_dis_bmp(void)
{
    int16 i,j;
    int16 temp,temp1;
    uint8 dat;


    temp1 = height_Inverse_Perspective%8;
    if(temp1 == 0) temp = height_Inverse_Perspective/8;
    else           temp = height_Inverse_Perspective/8+1;

    for(i=0; i<temp; i++)
    {
        oled_set_pos(0,(uint8)i);
        for(j=0; j<width_Inverse_Perspective; j++)
        {
            dat = 0;

            dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+0)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+0)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+0][j] == 1? 1: 0))<<0;
            if( i<(temp-1) || !temp1 || temp1>=2)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+1)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+1)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+1][j] == 1? 1: 0))<<1;
            if( i<(temp-1) || !temp1 || temp1>=3)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+2)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+2)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+2][j] == 1? 1: 0))<<2;
            if( i<(temp-1) || !temp1 || temp1>=4)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+3)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+3)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+3][j] == 1? 1: 0))<<3;
            if( i<(temp-1) || !temp1 || temp1>=5)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+4)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+4)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+4][j] == 1? 1: 0))<<4;
            if( i<(temp-1) || !temp1 || temp1>=6)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+5)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+5)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+5][j] == 1? 1: 0))<<5;
            if( i<(temp-1) || !temp1 || temp1>=7)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+6)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+6)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+6][j] == 1? 1: 0))<<6;
            if( i<(temp-1) || !temp1 || temp1>=8)dat |= (Col_Center[height_Inverse_Perspective - 1 - (i*8+7)]>=j-0.5 && Col_Center[height_Inverse_Perspective - 1 - (i*8+7)]<j+0.5?0:(mt9v03x_image_cutted_thresholding_inversePerspective[i*8+7][j] == 1? 1: 0))<<7;
            oled_wrdat(dat);
        }
    }
}
