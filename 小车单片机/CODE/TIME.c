#include "headfile.h"
#include "TIME.h"


uint32 time_100us[TIMER_NUM] = {0};
float time_up[TIMER_NUM];
enum TIMERSTATUS timer_Status[TIMER_NUM] = {PAUSED};

void My_Init_Timer(void)
{
    pit_interrupt_us(CCU6_0, PIT_CH0, 100);
}

void Start_Timer(uint8 ID)
{
    timer_Status[ID] = RUNNING;
}

void Pause_Timer(uint8 ID)
{
    timer_Status[ID] = PAUSED;
}

void Reset_Timer(uint8 ID)
{
    timer_Status[ID] = PAUSED;
    time_100us[ID] = 0;
}

void Set_Timer(uint8 ID, float time)
{
    time_100us[ID] = (uint32)(10000*time);
}

float Read_Timer(uint8 ID)
{
    return (time_100us[ID]/10000.0f);
}

int Read_Timer_Status(uint8 ID)
{
    return timer_Status[ID];
}

void Timer_Action_per100us(void)
{
    for (int ID=0;ID<TIMER_NUM;ID++)
    {
        switch (timer_Status[ID])
        {
            case RUNNING:
                time_100us[ID] = time_100us[ID] + 1;
                break;
            case PAUSED:
                break;
            default:
                break;
        }
    }

}

