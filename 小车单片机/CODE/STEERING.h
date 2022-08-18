#ifndef __STEERING_h__
#define __STEERING_h__

#define STEERING_MAX 50.0
#define STEERING_MIN -50.0
#define STEERING_DUTY_MAX 100.0//100.0//80.0//170.0

extern float steering_Target;
extern float steering_Error;
extern float d_steering_Error;
extern struct steerpid Steering_PID;
extern uint32 STEERING_DUTY_CENTER;
extern float SightForward;
extern float SightForward_Highest;
extern float SightForward_High;
extern float SightForward_Low;
extern float SightForward_Lowest;
extern float SightForward_Lowest_ForT;
extern float kp,kd;
extern uint8 TurnTime;

void My_Init_Steering(void);
void UART_Steering(void);
void Set_Steering_Target(uint8 val);
void UART_SteeringPID(void);
void Set_SteeringPID(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6);



void Set_Steering(void);
void Cal_Steering_Error(float Cal_Steering_Range_of_Img);
void Cal_Steering_Target(void);





#endif
