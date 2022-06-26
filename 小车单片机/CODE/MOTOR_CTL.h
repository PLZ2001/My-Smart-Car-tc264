#ifndef __MOTOR_CTL_h__
#define __MOTOR_CTL_h__

#define BANGBANG_UP 0.2
#define BANGBANG_DOWN 0.3

enum SpeedStatus
{
    Lowest,
    Low,
    High,
    Highest
};

extern float speed_Target;//目标速度
extern uint8 start_Flag;
extern uint8 emergency_Stop;
extern float speed_Target_Min;
extern float speed_Target_Max;
extern float speed_Target_Highest;
extern float speed_Target_Lowest;
extern float Differential_Ratio;
extern float InnerSide_Ratio;

extern float Base_Volt;
extern float Real_Volt;

extern enum SpeedStatus speed_Status;

void Differential_Motor(void);

#endif
