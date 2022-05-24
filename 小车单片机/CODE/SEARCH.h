#ifndef __SEARCH_h__
#define __SEARCH_h__

#ifndef width_Inverse_Perspective_Max
#define width_Inverse_Perspective_Max 128
#endif

#ifndef height_Inverse_Perspective_Max
#define height_Inverse_Perspective_Max 128
#endif

#define Camera_Height 0.2

#define STRAIGHT_CONDITION 5


extern float Col_Center[height_Inverse_Perspective_Max];//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
extern int Col_Left[height_Inverse_Perspective_Max];
extern int Col_Right[height_Inverse_Perspective_Max];
extern int search_Lines;

extern float threeRoads_RightTime;
extern float rightCircle_RightTime;
extern float rightCircle_LeftTime;
extern float rightCircle_BannedTime;
extern float T_Time;
extern int8 last_angle_down;
extern int8 last_angle_up;
extern int8 Circle_lines;

extern int8 first_Dot[2];
extern int8 second_Dot[2];
extern int8 third_Dot[2];
extern float arccosValue;


void UART_ColCenter(void);
void UART_ColLeft(void);
void UART_ColRight(void);

uint8 Check_Straight(void);
uint8 Check_Left_Straight(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio);
uint8 Check_Right_Straight(int8 max_d_Col_Right, int8 min_d_Col_Right, float ratio);
uint8 Check_LeftCircle(void);
uint8 Check_RightCircle(void);
void DrawCenterLine(void);
void DrawCenterLinewithConfig(float filter);
void DrawCenterLinewithConfig_RightBased(float filter);
void DrawCenterLinewithConfig_LeftBased(float filter);
void DrawCenterLinewithConfig_CrossRoad(void);
void Find_First_Dot(int mode);
void Find_Second_Dot(int mode);
void Find_Third_Dot(int mode);





#endif
