#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//��ʾ�����
#include "UART.h"
#include "MOTOR_CTL.h"


void My_Init_ICM(void)
{
    simiic_init();
    icm20602_init();
}

void Get_ICM_DATA(void)
{
    get_icm20602_gyro();
}
