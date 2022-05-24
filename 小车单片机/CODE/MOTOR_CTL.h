#ifndef __MOTOR_CTL_h__
#define __MOTOR_CTL_h__

#define BANGBANG_UP 0.1
#define BANGBANG_DOWN 0.3

extern float speed_Target;//目标速度
extern uint8 start_Flag;
extern uint8 emergency_Stop;
extern float speed_Target_Min;
extern float speed_Target_Max;
extern float Differential_Ratio;
extern float InnerSide_Ratio;

void Differential_Motor(void);

#endif
