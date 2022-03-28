#ifndef __SEARCH_h__
#define __SEARCH_h__


#define Camera_Height 0.2

#define STRAIGHT_CONDITION 5

#define road_width (0.4/Camera_Height/ratioOfPixelToHG) //��·ʵ�ʿ��0.4m


extern float Col_Center[height_Inverse_Perspective_Max];//���������ϵ�˳��洢�������ߵ��кŽ�������Ϸ���ȫ��Ϊ-2
extern int Col_Left[height_Inverse_Perspective_Max];
extern int Col_Right[height_Inverse_Perspective_Max];
extern int search_Lines;

void UART_ColCenter(void);
void UART_ColLeft(void);
void UART_ColRight(void);

uint8 Check_Straight(void);
uint8 Check_Left_Straight(void);
uint8 Check_RightCircle(void);
void DrawCenterLine(void);
void DrawCenterLinewithConfig(float filter);
void DrawCenterLinewithConfig_RightBased(float filter);
void DrawCenterLinewithConfig_LeftBased(float filter);
void DrawCenterLinewithConfig_CrossRoad(void);



#endif
