#ifndef __TIME_h__
#define __TIME_h__

#define TIMER_NUM 12
#define InsertTimer1Point(x)  if (OLED_Page == Timer_Page2){OLED_PRINTF(0,x,"NO%d: %.01f ms    ",x,1000*Read_Timer(1));Reset_Timer(1);Start_Timer(1);}
//#define InsertTimer1Point(x)  if (OLED_Page == Timer_Page2){oled_p6x8str(0,x,"NO");oled_uint16(2*6,x,x);oled_p6x8str(3*6,x,": ");oled_printf_float(5*6,x,1000*Read_Timer(1),2,1);oled_p6x8str(9*6,x,"ms");Reset_Timer(1);Start_Timer(1);}
#define InsertTimer2Point(x)  if (OLED_Page == Timer_Page2){OLED_PRINTF(0,x,"NO%d: %.01f ms    ",x,1000*Read_Timer(2));Reset_Timer(2);Start_Timer(2);}

enum TIMERSTATUS
{
    PAUSED,
    RUNNING
};

extern float time_up[TIMER_NUM];

void Start_Timer(uint8 ID);
void Pause_Timer(uint8 ID);
void Reset_Timer(uint8 ID);
void Set_Timer(uint8 ID, float time);
float Read_Timer(uint8 ID);
int Read_Timer_Status(uint8 ID);
void Timer_Action_per100us(void);
void My_Init_Timer(void);




#endif
