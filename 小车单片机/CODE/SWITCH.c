#include "headfile.h"
#include "SWITCH.h"

uint8 switch_Status[2] = {0,0};

void My_Init_Switch(void)
{
    gpio_init(SWITCH1_GPIO, GPI, 0, NO_PULL);
    gpio_init(SWITCH2_GPIO, GPI, 0, NO_PULL);
}

void Check_Switch_per10ms(void)
{
    uint8 switch1 = gpio_get(SWITCH1_GPIO);
    uint8 switch2 = gpio_get(SWITCH2_GPIO);
    switch_Status[Switch1]=switch1;
    switch_Status[Switch2]=switch2;

}
