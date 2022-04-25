#ifndef __KEY_h__
#define __KEY_h__

#define ZJU_PCB //�����ZJUһ�Ӱ��ӣ���д��仰���������������ӣ���ע�͵���仰

#ifdef ZJU_PCB

//���һ�Ӱ���
#define KEY0_GPIO P33_8
#define KEY1_GPIO P33_9
#define KEY2_GPIO P33_11

#else

//�������
#define KEY0_GPIO P22_0
#define KEY1_GPIO P22_1
#define KEY2_GPIO P22_2

#endif

#define STATUS0_COUNTER_MAX 5
#define STATUS1_COUNTER_MAX 50

enum Keycode
{
    Key0,
    Key1,
    Key2
};

void My_Init_Key(void);
void Check_Key_per10ms(void);
void Key0_Action(void);
void Key1_Action(void);
void Key2_Action(void);


#endif
