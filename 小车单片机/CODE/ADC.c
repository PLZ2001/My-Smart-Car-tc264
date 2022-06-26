#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//显示屏相关
#include "UART.h"
#include "MOTOR_CTL.h"


void My_Init_ADC(void)
{
    adc_init(ADC_0, ADC0_CH7_A7);//初始化ADC0 通道0 使用A0引脚
}

void Get_ADC_DATA(void)
{
    int adc_result = adc_mean_filter(ADC_0, ADC0_CH7_A7, ADC_12BIT, 10);//采集10次求平均  分辨率12位
    Real_Volt = adc_result/4095.0f*3.30f*((4.7f+1.0f)/1.0f);
}
