#ifndef __SEARCH_h__
#define __SEARCH_h__

enum ZEBRA_STATUS
{
    starting,
    finding,
    banning,
//    ready_finishing,
    finishing
};


#ifndef width_Inverse_Perspective_Max
#define width_Inverse_Perspective_Max 128
#endif

#ifndef height_Inverse_Perspective_Max
#define height_Inverse_Perspective_Max 128
#endif

#define Camera_Height 0.3

#define STRAIGHT_CONDITION 3

#define MAX_Row_Index 9

#define pos_num 8


extern float Col_Center[height_Inverse_Perspective_Max];//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
extern int Col_Left[height_Inverse_Perspective_Max];
extern int Col_Right[height_Inverse_Perspective_Max];
extern int search_Lines;
extern int road_width;
extern float left_empty,right_empty;


extern float threeRoads_RightTime;
extern float rightCircle_RightTime;
extern float rightCircle_LeftTime;
extern float rightCircle_BannedTime;
extern float T_Time;
extern int8 last_angle_down;
extern int8 last_angle_up;
extern int8 last_angle_upup;
extern int8 ThreeRoads_lines[2];
extern int8 Circle_lines;

extern int8 first_Dot[2];
extern int8 second_Dot[2];
extern int8 third_Dot[2];
extern float arccosValue;

extern uint8 DrawLineFilter;

extern float Left_Straight_Score,Unknown_Straight_Score,Right_Straight_Score;

extern uint8 flag_For_T;
extern uint8 flag_For_ThreeRoad;
extern uint8 White2Black_cnt;
extern enum ZEBRA_STATUS zebra_status;
extern uint8 Zebra_times;
extern uint8 Zebra_times_Max;
extern int zebra_direction;
extern int zebra_start_direction;

extern uint8 center_dot;
extern uint8 Zebra_Value;
extern float Zebra_Detect;

extern uint8 Helper_Window_Flag;
extern uint8 Check_Circle_New4_EN;

extern uint8 Stop;
extern float T_Line;

extern uint8 ThreeeRoad_Delay_Flag;
extern float ThreeeRoad_Delay;

extern int left_width[pos_num];
extern int right_width[pos_num];
extern float pos[pos_num];

extern uint8 rightCircle_Alarm;
extern uint8 leftCircle_Alarm;
extern uint8 crossRoad_Alarm;
extern uint8 rightCircle_Size;//2小圆，1大圆，0未知
extern uint8 leftCircle_Size;//2小圆，1大圆，0未知
extern uint8 crossRoad_Distance;


void UART_ColCenter(void);
void UART_ColLeft(void);
void UART_ColRight(void);

uint8 Check_Straight(float ratio);
uint8 Check_Left_Straight(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio);
uint8 Check_Right_Straight(int8 max_d_Col_Right, int8 min_d_Col_Right, float ratio);
uint8 Check_LeftCircle(void);
uint8 Check_RightCircle(void);
void DrawCenterLine(void);
void DrawCenterLinewithConfig(float filter);
void DrawCenterLinewithConfig_ThreeRoad(void);
void DrawCenterLinewithConfig_RightBased(float filter);
void DrawCenterLinewithConfig_LeftBased(float filter);
void DrawCenterLinewithConfig_CrossRoad(void);
void Find_First_Dot(int mode);
void Find_Second_Dot(int mode);
void Find_Third_Dot(int mode);

float Filter_Col_Left(uint8 flag,float value,float ratio);
float Filter_Col_Right(uint8 flag,float value,float ratio);
float min(float a,float b);
float max(float a,float b);

uint8 Check_Left_All_Road(float ratio,int zero_limit);
uint8 Check_Right_All_Road(float ratio,int zero_limit);

float Get_Straight_Score(int dot_num);
uint8 Select_Left_Unknown_or_Right(int dot_num);

uint8 Check_LeftCircle_New4(float ratio);
uint8 Check_RightCircle_New4(float ratio);

uint8 Check_Far_Road_And_Draw(int mode,float ratio);





#endif
