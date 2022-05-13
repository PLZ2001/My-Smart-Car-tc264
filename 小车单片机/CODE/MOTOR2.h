#ifndef __MOTOR2_h__
#define __MOTOR2_h__

//2表示左电机

#define SPEED_MEASURING_PERIOD_ms2 10
#define SPEED_MAX2 7.0
#define SPEED_MIN2 -7.0
#define MOTOR_DUTY_MAX2 5000

enum PID_Mode2
{
    OPEN_LOOP2,
    PID_CLOSED_LOOP2,
    FUZZY_PID_CLOSED_LOOP2
};

extern float speed_Measured2;
extern float speed_Target2;
extern float speed_Output2;
extern enum PID_Mode2 PID_mode2;
extern float PID_KP2;
extern float PID_KI2;
extern float PID_KD2;

void UART_Speed2(void);
void Set_Speed_Target2(uint8 val);
void UART_PID2(void);
void Set_PID2(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6);


void My_Init_Motor2(void);
void My_Init_SpeedSensor2(void);
void Get_Speed_perSPEED_MEASURING_PERIOD_ms2(void);
//void Get_Speed(void);
void Cal_Speed_Output2(void);
void Set_Speed2(void);
void Cal_Speed_Target2(void);



#endif
