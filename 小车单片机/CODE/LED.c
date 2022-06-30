#include "headfile.h"
#include "LED.h"


void My_Init_LED(void)
{
    gpio_init(GPIO_LED1, GPO, 1, PUSHPULL);//LED1
    gpio_init(GPIO_LED2, GPO, 1, PUSHPULL);//LED1
}

void LED_ON(uint8 ID)
{
    switch(ID)
    {
        case 1:
        {
            gpio_set(GPIO_LED1,0);
            break;
        }
        case 2:
        {
            gpio_set(GPIO_LED2,0);
            break;
        }
        default:
        {
            break;
        }
    }
}

void LED_OFF(uint8 ID)
{
    switch(ID)
    {
        case 1:
        {
            gpio_set(GPIO_LED1,1);
            break;
        }
        case 2:
        {
            gpio_set(GPIO_LED2,1);
            break;
        }
        default:
        {
            break;
        }
    }
}
