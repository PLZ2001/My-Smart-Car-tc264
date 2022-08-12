#include "headfile.h"
#include "ICM.h"
#include "CAMERA.h"
#include "TIMETIME.h"
#include "STEERING.h"

uint8 flag_for_ICM_Init=0;

float my_acc_x;
float my_acc_y;
float my_acc_z;
float my_gyro_x;
float my_gyro_y;
float my_gyro_z;

//float icm_acc_x_bias;
//float icm_acc_y_bias;
//float icm_acc_z_bias;
float icm_gyro_x_bias;
float icm_gyro_y_bias;
float icm_gyro_z_bias;

float angle=0;

uint8 is_Slope = 0;//1��ʾ��⵽�µ���0��ʾû��

float Lazer_Data=819.1f;
uint8 Lazer_On = 1;

float SlopeSpeed1 = 0.6f;//0.4f;
float SlopeSpeed2 = 0.6f;//0.4f;
float SlopeSpeed3 = 0.7f;

float Slope_Distance = 70.0f;

uint8 Max_Limit = 20;//30;
float Lazer_Start_Time = 0;
float Lazer_End_Time = 36.0f;

void My_Init_ICM(void)
{
    icm20602_init();
}

void Get_ICM_DATA(void)
{
//    get_icm20602_accdata();
//    my_acc_x = Filter(2,icm_acc_x,0.01)/4096.0f;
//    my_acc_y = Filter(3,icm_acc_y,0.01)/4096.0f;
//    my_acc_z = Filter(4,icm_acc_z,0.01)/4096.0f;
    get_icm20602_gyro();
//    my_gyro_x = Filter(5,icm_gyro_x-icm_gyro_x_bias,0.01)/16.4f;
    my_gyro_y = Filter(6,icm_gyro_y-icm_gyro_y_bias,0.01)/16.4f;
//    my_gyro_z = Filter(7,icm_gyro_z-icm_gyro_z_bias,0.01)/16.4f;
}

void UART_ICM(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x14);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    int16 gyro_y = (int16)round(100*my_gyro_y);
    int16 acc_z = (int16)round(10000*my_acc_z);
    uart_putchar(DEBUG_UART, gyro_y>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, gyro_y&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, acc_z>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, acc_z&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x14);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void Get_Zero_Bias(void)
{
//    float acc[3]={0,0,0};
    float gyro[3]={0,0,0};
    uint8 times=100;
    uint8 T_ms=5;
    for (int time=0;time<times;time++)
    {
//        get_icm20602_accdata();
//        acc[0]+=icm_acc_x;
//        acc[1]+=icm_acc_y;
//        acc[2]+=icm_acc_z;
        get_icm20602_gyro();
        gyro[0]+=icm_gyro_x;
        gyro[1]+=icm_gyro_y;
        gyro[2]+=icm_gyro_z;
        systick_delay_ms(STM0, T_ms);
    }
//    icm_acc_x_bias = acc[0]/times;
//    icm_acc_y_bias = acc[1]/times;
//    icm_acc_z_bias = acc[2]/times;
    icm_gyro_x_bias = gyro[0]/times;
    icm_gyro_y_bias = gyro[1]/times;
    icm_gyro_z_bias = gyro[2]/times;
}

void Check_Slope_with_Gyro(void)
{
    if (Read_Timer_Status(10) == PAUSED)
    {
        if(my_gyro_y>GYRO_Y_VALUE)
        {
            is_Slope = 1;
            time_up[9] = 0.2f;
            Start_Timer(9);
        }
        if (Read_Timer_Status(9) == RUNNING)
        {
            is_Slope = 1;
            if (Read_Timer(9)>time_up[9])
            {
                is_Slope = 0;
                Reset_Timer(9);
                Reset_Timer(10);
                time_up[10] = 2.0f;
                Start_Timer(10);
            }
        }
        else
        {
            is_Slope = 0;
        }
    }
    else
    {
        if (Read_Timer(10)>time_up[10])
        {
            Reset_Timer(10);
        }
    }

}

void Check_Slope_with_Lazer(void)
{
    static int cnt_0 = 0;
    if(is_Slope == 0 && Read_Timer(12) == PAUSED && Check_Fake_Slope(1))
    {
        if (Lazer_Data<Slope_Distance)
        {
            cnt_0++;
            if (cnt_0>=1)
            {
                is_Slope = 1;//��������
                cnt_0=0;
            }
        }
        else
        {
            cnt_0 = 0;
        }
    }

    static int cnt = 0;
    if (is_Slope == 1)
    {
        if (Lazer_Data>800.0f)
        {
            cnt++;
            if (cnt>=3)
            {
                cnt = 0;
                is_Slope = 2;//����ƽ��
            }
        }
        else
        {
            cnt = 0;
        }
    }

    if (is_Slope == 2)
    {
        if (Lazer_Data<70.0f)
        {
            is_Slope = 3;//��������
        }
    }

    static int cnt_1 = 0;
    if (is_Slope == 3)
    {
//        if (steering_Target<20 && steering_Target>-20)
        if (Lazer_Data>800.0f)
        {
            cnt_1++;
            if (cnt_1>=3)
            {
                is_Slope = 0;//�뿪�µ�
                time_up[12] = 0.5f;
                Start_Timer(12);
            }
        }
        else
        {
            cnt_1 = 0;
        }
    }

    static int cnt_2 = 0;
    if (Lazer_Data>800.0f)
    {
        cnt_2++;
        if (cnt_2>=Max_Limit)
        {
            cnt_2 = 0;
            is_Slope = 0;//�Ѿ����뵽������·
            time_up[12] = 0.5f;
            Start_Timer(12);
        }
    }
    else
    {
        cnt_2 = 0;
    }

    if (Read_Timer(12)>time_up[12])
    {
        Reset_Timer(12);
    }

}

