#ifndef __ICM_h__
#define __ICM_h__

extern uint8 flag_for_ICM_Init;

extern float my_acc_x;
extern float my_acc_y;
extern float my_acc_z;
extern float my_gyro_x;
extern float my_gyro_y;
extern float my_gyro_z;

void My_Init_ICM(void);
void Get_ICM_DATA(void);



#endif
