#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//��ʾ�����
#include "UART.h"
#include "MOTOR_CTL.h"


void My_Init_ADC(void)
{
    adc_init(ADC_0, ADC0_CH7_A7);//��ʼ��ADC0 ͨ��0 ʹ��A0����
}

void Get_ADC_DATA(void)
{
    int adc_result = adc_mean_filter(ADC_0, ADC0_CH7_A7, ADC_12BIT, 10);//�ɼ�10����ƽ��  �ֱ���12λ
    Real_Volt = adc_result/4095.0f*3.30f*((4.7f+1.0f)/1.0f);
}
