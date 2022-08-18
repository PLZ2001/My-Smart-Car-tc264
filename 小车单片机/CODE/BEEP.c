#include "headfile.h"
#include "BEEP.h"
#include "TIME.h"

float BEEP_Time_s = 0;
int BEEP_Times = 0;
enum BEEP_STATUS BEEP_Status = STOP;

void My_Init_BEEP(void)
{
    gpio_init(P15_5, GPO, 0, PUSHPULL);
}

void BEEP(float time_s,int times)
{
    BEEP_Time_s = time_s;
    BEEP_Times = times;
    BEEP_Status = START;
}


void BEEP_Action(void)
{
    static int cnt = 0;
    switch (BEEP_Status)
    {
        case START:
        {
            cnt = 0;
            time_up[19]=0;
            Reset_Timer(19);
            gpio_set(P15_5, 0);
            BEEP_Status = RUNNING_BEEP_ON;
            break;
        }
        case RUNNING_BEEP_ON:
        {
            time_up[19] = BEEP_Time_s;
            Start_Timer(19);
            if (Read_Timer(19)>time_up[19])
            {
                Reset_Timer(19);
                cnt++;
                if (cnt<BEEP_Times)
                {
                    BEEP_Status = RUNNING_BEEP_OFF;
                }
                else
                {
                    BEEP_Status = STOP;
                }
            }
            else
            {
                gpio_set(P15_5, 1);
            }
            break;
        }
        case RUNNING_BEEP_OFF:
        {
            time_up[19] = BEEP_Time_s;
            Start_Timer(19);
            if (Read_Timer(19)>time_up[19])
            {
                Reset_Timer(19);
                BEEP_Status = RUNNING_BEEP_ON;
            }
            else
            {
                gpio_set(P15_5, 0);
            }
            break;
        }
        case STOP:
        default:
        {
            cnt = 0;
            time_up[19]=0;
            Reset_Timer(19);
            gpio_set(P15_5, 0);
            break;
        }
    }
}
