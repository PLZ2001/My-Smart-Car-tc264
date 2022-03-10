#ifndef __STEERING_h__
#define __STEERING_h__

#define STEERING_MAX 32.0
#define STEERING_MIN -32.0
#define STEERING_DUTY_MAX 381.0

extern float steering_Target;

void My_Init_Steering(void);
void UART_Steering(void);
void Set_Steering_Target(uint8 val);

void Set_Steering(void);
void Cal_Steering_Error(float Cal_Steering_Range_of_Img);
void Cal_Steering_Target(void);





#endif
