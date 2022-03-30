#include "headfile.h"
#include "TIME.h"

uint32 time_1ms = 0;
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
    time_1ms = 0;
}

void Set_Timer(float time)
{
    time_1ms = (uint32)(1000*time);
}

float Read_Timer(void)
{
    return (time_1ms/1000.0f);
}

int Read_Timer_Status(void)
{
    return timer_Status;
}

void Timer_Action_per1ms(void)
{
    switch (timer_Status)
    {
        case RUNNING:
            time_1ms = time_1ms + 1;
            break;
        case PAUSED:
            break;
        default:
            break;
    }
}

