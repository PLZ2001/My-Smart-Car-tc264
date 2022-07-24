#include "headfile.h"
#include "TIMETIME.h"

uint8 timetime_flag=0;
uint32 time_10ms[TIMER_NUM] = {0};
float time_up[TIMER_NUM];
enum TIMERSTATUS timer_Status[TIMER_NUM] = {PAUSED};

void My_Init_Timer(void)
{
    pit_interrupt_ms(CCU6_0, PIT_CH0, 10);
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
    time_10ms[ID] = 0;
}

void Set_Timer(uint8 ID, float time)
{
    time_10ms[ID] = (uint32)(100*time);
}

float Read_Timer(uint8 ID)
{
    return (time_10ms[ID]/100.0f);
}

int Read_Timer_Status(uint8 ID)
{
    return timer_Status[ID];
}

void Timer_Action_per10ms(void)
{
    for (int ID=0;ID<TIMER_NUM;ID++)
    {
        switch (timer_Status[ID])
        {
            case RUNNING:
                time_10ms[ID] = time_10ms[ID] + 1;
                break;
            case PAUSED:
                break;
            default:
                break;
        }
    }

}

