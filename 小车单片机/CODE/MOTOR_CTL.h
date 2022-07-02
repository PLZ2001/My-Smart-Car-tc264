#ifndef __MOTOR_CTL_h__
#define __MOTOR_CTL_h__


enum SpeedMode
{
    Lowest_Mode,
    Low_Mode,
    High_Mode,
    Highest_Mode
};

enum SpeedStatus
{
    Lowest_ForT,
    Lowest,
    Low,
    High,
    Highest
};

extern float speed_Target;//目标速度
extern uint8 start_Flag;
extern uint8 emergency_Stop;
extern float speed_Target_Low;
extern float speed_Target_High;
extern float speed_Target_Highest;
extern float speed_Target_Lowest;
extern float speed_Target_Lowest_ForT;
extern float Differential_Ratio;
extern float InnerSide_Ratio;
extern float InnerSide_Ratio_Highest;
extern float InnerSide_Ratio_High;
extern float InnerSide_Ratio_Low;
extern float InnerSide_Ratio_Lowest;
extern float InnerSide_Ratio_Lowest_ForT;
extern float Steering_PID_Highest[3];
extern float Steering_PID_High[3];
extern float Steering_PID_Low[3];
extern float Steering_PID_Lowest[3];
extern float Steering_PID_Lowest_ForT[3];

extern float Base_Volt;
extern float Real_Volt;

extern enum SpeedStatus speed_Status;
extern enum SpeedMode speed_Mode;

extern float BANGBANG_UP;
extern float BANGBANG_DOWN;

extern float Highest_Distance;


void Differential_Motor(void);

#endif
