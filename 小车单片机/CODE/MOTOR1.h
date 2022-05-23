#ifndef __MOTOR1_h__
#define __MOTOR1_h__

#define SPEED_MEASURING_PERIOD_ms1 10
#define SPEED_MAX1 9.5
#define SPEED_MIN1 -9.5
#define MOTOR_DUTY_MAX1 6750

enum PID_Mode1
{
    OPEN_LOOP1,
    PID_CLOSED_LOOP1,
    FUZZY_PID_CLOSED_LOOP1,
    BANGBANG_CLOSED_LOOP1
};

extern float speed_Measured1;
extern float speed_Target1;
extern float speed_Output1;
extern enum PID_Mode1 PID_mode1;
extern float PID_KP1;
extern float PID_KI1;
extern float PID_KD1;


void UART_Speed1(void);
void Set_Speed_Target1(uint8 val);
void UART_PID1(void);
void Set_PID1(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6);


void My_Init_Motor1(void);
void My_Init_SpeedSensor1(void);
void Get_Speed_perSPEED_MEASURING_PERIOD_ms1(void);
//void Get_Speed(void);
void Cal_Speed_Output1(void);
void Set_Speed1(void);
void Cal_Speed_Target1(void);



#endif
