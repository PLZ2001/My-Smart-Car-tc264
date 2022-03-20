#include "headfile.h"
#include "TIME.h"

uint32 time_10ms = 0;
float time_up;
enum TIMER_STATUS timer_Status = PAUSED;

void Start_Timer(void)
{
    timer_Status = RUNNING;
}

void Pause_Timer(void)
{
    timer_Status = PAUSED;
}

void Reset_Timer(void)
{
    timer_Status = PAUSED;
    time_10ms = 0;
}

void Set_Timer(float time)
{
    time_10ms = (uint32)(100*time);
}

float Read_Timer(void)
{
    return (time_10ms/100.0f);
}

int Read_Timer_Status(void)
{
    return timer_Status;
}

void Timer_Action_per10ms(void)
{
    switch (timer_Status)
    {
        case RUNNING:
            time_10ms = time_10ms + 1;
            break;
        case PAUSED:
            break;
        default:
            break;
    }
}

