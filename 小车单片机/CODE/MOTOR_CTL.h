#ifndef __MOTOR_CTL_h__
#define __MOTOR_CTL_h__


extern float speed_Target;//目标速度
extern uint8 start_Flag;
extern uint8 emergency_Stop;
extern float speed_Target_Min;
extern float speed_Target_Max;
extern float Differential_Ratio;

void Differential_Motor(void);

#endif
