#include "headfile.h"
#include "ICM.h"
#include "CAMERA.h"

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

uint8 is_Slope = 0;//1表示检测到坡道，0表示没有

void My_Init_ICM(void)
{
    icm20602_init();
}

void Get_ICM_DATA(void)
{
    get_icm20602_accdata();
    my_acc_x = Filter(2,icm_acc_x,0.01)/4096.0f;
    my_acc_y = Filter(3,icm_acc_y,0.01)/4096.0f;
    my_acc_z = Filter(4,icm_acc_z,0.01)/4096.0f;
    get_icm20602_gyro();
    my_gyro_x = Filter(5,icm_gyro_x-icm_gyro_x_bias,0.01)/16.4f;
    my_gyro_y = Filter(6,icm_gyro_y-icm_gyro_y_bias,0.01)/16.4f;
    my_gyro_z = Filter(7,icm_gyro_z-icm_gyro_z_bias,0.01)/16.4f;
}

void UART_ICM(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x14);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    int16 gyro_y = (int16)round(100*my_gyro_y);
    int16 acc_z = (int16)round(10000*my_acc_z);
    uart_putchar(DEBUG_UART, gyro_y>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, gyro_y&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, acc_z>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, acc_z&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x14);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
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

uint8 Check_Slope_with_YHF(void)
{
    if(my_gyro_y>GYRO_Y_VALUE)
    {
        is_Slope = 1;
    }
    else
    {
        is_Slope = 0;
    }
}

uint8 Check_SLope_with_PLZ(void)
{
    //校正零漂
//    if (my_acc_z)
}
