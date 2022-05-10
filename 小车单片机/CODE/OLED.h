#ifndef __OLED_h__
#define __OLED_h__

#define OLED_PRINTF(x,y,...) {char string[40];sprintf(string, __VA_ARGS__);oled_p6x8str(x,y,string);}
//������������printfһ����OLED�������ʽ���ַ���
//���磺OLED_PRINTF(0,0,"The speed is %d km/s", speed);
#define PAGE_NUM 13

enum OLEDPage
{
    UART_Debug_Page,
    Zebra_Page,
    Circle_Page,
    Steering_Center_Page,
    Camera_Page,
    SteeringPID_Page,
    TimeSet_Page,
    Speed_Page,
    UART_Setting_Page,
    OLED_Setting_Page,
    Timer_Page,
    Gyroscope_Page,
    Timer_Page2
};

extern enum OLEDPage OLED_Page;
extern uint8 OLED_EN;
extern uint8 OLED_Page_Active_Flag;
extern uint8 pointer_temp;
extern int8 up_Down;

void My_Init_OLED(void);
void Update_OLED_per10ms(void);



#endif
