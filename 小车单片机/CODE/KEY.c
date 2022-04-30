#include "headfile.h"
#include "KEY.h"
#include "OLED.h"//��ʾ�����
#include "UART.h"
#include "MOTOR_CTL.h"
#include "SEARCH.h"
#include "CAMERA.h"



void My_Init_Key(void)
{
    gpio_init(KEY0_GPIO, GPI, 0, NO_PULL);//KEY0����
    gpio_init(KEY1_GPIO, GPI, 0, NO_PULL);//KEY1����
    gpio_init(KEY2_GPIO, GPI, 0, NO_PULL);//KEY2����
}

//������ʼ״̬Ϊ0
//���״̬0��״̬1��ĳ�����͵�ƽ��������һ
//���״̬0��״̬1��ĳ����һ���ߵ�ƽ����ص�״̬0����������
//�������״̬0��ĳ���������ﵽFIRST_COUNTER_MAX�������״̬1����ִ����Ӧ��������������
//�������״̬1��ĳ���������ﵽSECOND_COUNTER_MAX��������״̬1����ִ����Ӧ��������������
//�ú���Ҫ��10msִ��һ��
void Check_Key_per10ms(void)
{
    static uint8 counter[3], status[3], trigger[3];
    //���Key0
    if (gpio_get(KEY0_GPIO)==0)
    {
        counter[Key0]++;
        if (counter[Key0]>=(status[Key0]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key0] = 1;
            trigger[Key0] = TRUE;
            counter[Key0] = 0;
        }
    }
    else
    {
        status[Key0] = 0;
        counter[Key0] = 0;
    }
    //���Key1
    if (gpio_get(KEY1_GPIO)==0)
    {
        counter[Key1]++;
        if (counter[Key1]>=(status[Key1]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key1] = 1;
            trigger[Key1] = TRUE;
            counter[Key1] = 0;
        }
    }
    else
    {
        status[Key1] = 0;
        counter[Key1] = 0;
    }
    //���Key2
    if (gpio_get(KEY2_GPIO)==0)
    {
        counter[Key2]++;
        if (counter[Key2]>=(status[Key2]==0?STATUS0_COUNTER_MAX:STATUS1_COUNTER_MAX))
        {
            status[Key2] = 1;
            trigger[Key2] = TRUE;
            counter[Key2] = 0;
        }
    }
    else
    {
        status[Key2] = 0;
        counter[Key2] = 0;
    }
    //ִ�ж���
    if (trigger[Key0])
    {
        Key0_Action();
        trigger[Key0] = FALSE;
    }
    if (trigger[Key1])
    {
        Key1_Action();
        trigger[Key1] = FALSE;
    }
    if (trigger[Key2])
    {
        Key2_Action();
        trigger[Key2] = FALSE;
    }
}

void Key0_Action(void)
{
    if (OLED_EN)
    {
        OLED_Page++;
        if (OLED_Page >= PAGE_NUM)
        {
            OLED_Page = 0;
        }
    }
    OLED_Page_Active_Flag = TRUE;
}

void Key1_Action(void)
{
    switch (OLED_Page)
    {
        case UART_Setting_Page:
            UART_EN = UART_EN?FALSE:TRUE;
            break;
        case OLED_Setting_Page:
            OLED_EN = OLED_EN?FALSE:TRUE;
            break;
        case TimeSet_Page:
            pointer_temp += 1;
            if (pointer_temp >= 4)
            {
                pointer_temp = 0;
                up_Down = -up_Down;
            }
            break;
        case Speed_Page:
            start_Flag = start_Flag?0:1;
            break;
        default:
            break;
    }
    OLED_Page_Active_Flag = TRUE;
}

void Key2_Action(void)
{
    switch (OLED_Page)
    {
        case Camera_Page:{
            static int8 direction = 1;
            fuzzy_thresholdingValue_36+=0.05*direction;
            if (fuzzy_thresholdingValue_36>=1)
            {
                direction = -1;
            }
            if (fuzzy_thresholdingValue_36<=0)
            {
                direction = 1;
            }
            break;}
        case UART_Setting_Page:
            UART_EN = UART_EN?FALSE:TRUE;
            break;
        case OLED_Setting_Page:
            OLED_EN = OLED_EN?FALSE:TRUE;
            break;
        case TimeSet_Page:
            switch (pointer_temp)
            {
                case 0:
                    threeRoads_RightTime += 0.01f*up_Down;
                    break;
                case 1:
                    rightCircle_RightTime += 0.1f*up_Down;
                    break;
                case 2:
                    rightCircle_LeftTime += 0.1f*up_Down;
                    break;
                case 3:
                    rightCircle_BannedTime += 0.5f*up_Down;
                    break;
                default:
                    break;
            }
            break;
        case Speed_Page:{
            static int8 direction = 1;
            speed_Target+=0.2*direction;
            if (speed_Target>=3)
            {
                direction = -1;
            }
            if (speed_Target<=-3)
            {
                direction = 1;
            }
            break;}
        default:
            break;
    }
    OLED_Page_Active_Flag = TRUE;
}
