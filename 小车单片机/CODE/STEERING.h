#ifndef __STEERING_h__
#define __STEERING_h__

#define STEERING_MAX 40.0
#define STEERING_MIN -40.0
#define STEERING_DUTY_MAX 170.0
#define STEERING_DUTY_CENTER 1530

extern float steering_Target;

void My_Init_Steering(void);
void UART_Steering(void);
void Set_Steering_Target(uint8 val);
void UART_SteeringPID(void);
void Set_SteeringPID(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6);



void Set_Steering(void);
void Cal_Steering_Error(float Cal_Steering_Range_of_Img);
void Cal_Steering_Target(void);





#endif
