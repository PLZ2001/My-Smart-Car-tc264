#ifndef __OLED_h__
#define __OLED_h__

#define OLED_PRINTF(x,y,...) {char string[40];sprintf(string, __VA_ARGS__);oled_p6x8str(x,y,string);}
//可以像正常的printf一样在OLED上输出格式化字符串
//例如：OLED_PRINTF(0,0,"The speed is %d km/s", speed);
#define PAGE_NUM 16//21

enum OLEDPage
{
    Camera_Page2,
    Width_Page,
    Steering_Center_Page,
    Lazer_Page,
    Zebra_Page,
    Slope_Page,
    DR_Page,
    Volt_Page,
    Speed_Page,

//    Zebra_Page2,

//    Differential_Page,
//    MotorPID_Page,
//    SteeringPID_Page,


    Speed_Page2,
    ThreeDot_Page,
//    Circle_Page,
    Camera_Page,
    TimeSet_Page,
    UART_Setting_Page,
//    OLED_Setting_Page,
//    Timer_Page,
    Gyroscope_Page,
    Timer_Page2,
//    UART_Debug_Page
};

extern enum OLEDPage OLED_Page;
extern uint8 OLED_EN;
extern uint8 OLED_Page_Active_Flag;
extern uint8 pointer_temp;
extern int8 up_Down;
extern uint8 OLED_Camera_flag;

void My_Init_OLED(void);
void Update_OLED_per10ms(void);
void my_oled_dis_bmp(void);


#endif
