#ifndef __SWITCH_h__
#define __SWITCH_h__


//浙大一队板子
#define SWITCH1_GPIO P00_8
#define SWITCH2_GPIO P00_12


enum SwitchCode
{
    Switch1,
    Switch2,
};

void My_Init_Switch(void);
void Check_Switch_per10ms(void);

extern uint8 switch_Status[2];


#endif
