#ifndef __CAMERA_h__
#define __CAMERA_h__

#define Y_WIDTH_CAMERA 40
#define X_WIDTH_CAMERA 187

#define width_Inverse_Perspective_Max 256
#define height_Inverse_Perspective_Max 256

#define Camera_Height 0.2

#define CLASS_NUM 6

#define CLASSIFICATION_16_VALID 25//35太大了//21不够大
#define CLASSIFICATION_25_VALID 37//40太大了//35不够大

//曾经的分4类
//#define KMEANS_K 4
//#define GOD_LIGHT 205

#define KMEANS_K 2
#define GOD_LIGHT 205

#define STRAIGHT_CONDITION 5

#define road_width (0.4/Camera_Height/ratioOfPixelToHG) //道路实际宽度0.4m

extern int width_Inverse_Perspective;
extern int height_Inverse_Perspective;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image_cutted[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];
extern uint8 thresholding_Value;
extern uint8 mt9v03x_image_cutted_thresholding_inversePerspective[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
extern float cameraAlphaUpOrDown;//无需校正
extern float cameraThetaDown;//需要校正
extern float ratioOfMaxDisToHG;//仅影响显示距离
extern float ratioOfPixelToHG;//仅影响分辨率
extern uint8 classification_Result;

extern float Col_Center[height_Inverse_Perspective_Max];//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
extern int search_Lines;

void My_Init_Camera(void);

void UART_Image(void);
void UART_Thresholding_Image(void);
void UART_Thresholding_Value(void);
void UART_Inverse_Perspective(void);
void UART_Classification(void);
void UART_ColCenter(void);
void Set_Thresholding_Value(uint8 val);
void Set_CameraAlphaUpOrDown(uint8 val);
void Set_CameraThetaDown(uint8 val);
void Set_RatioOfMaxDisToHG(uint8 val);
void Set_RatioOfPixelToHG(uint8 val);
uint8 Classification_16(void);
uint8 Classification_25(void);
int Check_Straight(void);
int Check_Left_Straight(void);
void DrawCenterLine(void);
void DrawCenterLinewithConfig(float filter);
void DrawCenterLinewithConfig_RightBased(float filter);
void DrawCenterLinewithConfig_LeftBased(float filter);
void DrawCenterLinewithConfig_CrossRoad(void);
void Check_Classification(uint8 classification_Result_tmp, uint8 check_counter);






void Get_Cutted_Image(void);
void Get_Thresholding_Image(void);
void Get_Inverse_Perspective_Image(void);





#endif
