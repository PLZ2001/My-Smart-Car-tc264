#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//œ‘ æ∆¡œ‡πÿ
#include "UART.h"
#include "MOTOR_CTL.h"


void My_Init_ICM(void)
{
    icm20602_init();
}

void Get_ICM_DATA(void)
{
    get_icm20602_accdata();
    get_icm20602_gyro();
}
