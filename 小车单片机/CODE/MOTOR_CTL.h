#ifndef __MOTOR_CTL_h__
#define __MOTOR_CTL_h__

#define BANGBANG_UP 0.2
#define BANGBANG_DOWN 0.3

enum SpeedMode
{
    Lowest_Mode,
    Low_Mode,
    High_Mode,
    Highest_Mode
};

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
extern float speed_Target_Low;
extern float speed_Target_High;
extern float speed_Target_Highest;
extern float speed_Target_Lowest;
extern float Differential_Ratio;
extern float InnerSide_Ratio;
extern float InnerSide_Ratio_Highest;
extern float InnerSide_Ratio_High;
extern float InnerSide_Ratio_Low;
extern float InnerSide_Ratio_Lowest;
extern float Steering_PID_Highest[3];
extern float Steering_PID_High[3];
extern float Steering_PID_Low[3];
extern float Steering_PID_Lowest[3];

extern float Base_Volt;
extern float Real_Volt;

extern enum SpeedStatus speed_Status;
extern enum SpeedMode speed_Mode;

void Differential_Motor(void);

#endif
