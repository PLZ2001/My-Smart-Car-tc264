#ifndef __ICM_h__
#define __ICM_h__

#define GYRO_Y_VALUE 60.0f
#define ACC_Z_VALUE 0.25f

extern uint8 flag_for_ICM_Init;
extern uint8 is_Slope;


extern float my_acc_x;
extern float my_acc_y;
extern float my_acc_z;
extern float my_gyro_x;
extern float my_gyro_y;
extern float my_gyro_z;

extern float Lazer_Data;

extern uint8 Lazer_On;

extern float SlopeSpeed1;
extern float SlopeSpeed2;
extern float SlopeSpeed3;

extern float Slope_Distance;

extern uint8 Max_Limit;
extern float Lazer_Start_Time;
extern float Lazer_End_Time;

void My_Init_ICM(void);
void Get_ICM_DATA(void);
void Check_Slope_with_Gyro(void);
void Check_Slope_with_Lazer(void);



#endif
