#ifndef __TIME_h__
#define __TIME_h__

enum TIMER_STATUS
{
    RUNNING,
    PAUSED
};

void Start_Timer(void);
void Pause_Timer(void);
void Reset_Timer(void);
void Set_Timer(float time);
float Read_Timer(void);
void Timer_Action_per10ms(void);
int Read_Timer_Status(void);





#endif
