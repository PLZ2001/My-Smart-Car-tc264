#ifndef __CAMERA_h__
#define __CAMERA_h__

#define Y_WIDTH_CAMERA 120
#define X_WIDTH_CAMERA 187

#ifndef width_Inverse_Perspective_Max
#define width_Inverse_Perspective_Max 128
#endif

#ifndef height_Inverse_Perspective_Max
#define height_Inverse_Perspective_Max 64
#endif

#define CLASS_NUM 6
#define CLASS_NUM_NEW 7

//#define CLASSIFICATION_16_VALID 25//35太大了//21不够大
#define CLASSIFICATION_25_VALID 37//40太大了//35不够大

//曾经的分4类
//#define KMEANS_K 4
//#define GOD_LIGHT 205

#define KMEANS_K 2
#define GOD_LIGHT 205

extern int width_Inverse_Perspective;
extern int height_Inverse_Perspective;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
//extern uint8 mt9v03x_image_cutted[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];
extern uint8 thresholding_Value;
extern uint8 mt9v03x_image_cutted_thresholding_inversePerspective[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
extern float cameraAlphaUpOrDown;//无需校正
extern float cameraThetaDown;//需要校正
extern float ratioOfMaxDisToHG;//仅影响显示距离
extern float ratioOfPixelToHG;//仅影响分辨率
extern uint8 classification_Result;
extern uint8 classification_Result_2nd;
extern uint8 Thresholding_Value_Init_Flag;//表示是否初始化了二值化阈值
extern uint8 fuzzy_Image_25[5][5];
extern uint8 fuzzy_Image_36[6][6];
extern float fuzzy_thresholdingValue_25;
extern float fuzzy_thresholdingValue_36;
extern float score[CLASS_NUM_NEW];
extern float max_Score;

extern uint8 flag_For_Right_Circle;
extern uint8 flag_For_Left_Circle;
extern uint8 flag_For_Right_T;
extern uint8 flag_For_Left_T;

extern uint8 Search_Range[2][2];
extern uint8 Long_Straight_Flag;
extern uint8 classification_Result_1;
extern uint8 classification_Result_1_2nd;
extern uint8 classification_Result_2;
extern uint8 classification_Result_2_2nd;

extern float ModelTable_36[CLASS_NUM_NEW][6][6];

extern int invalid_lines;

extern uint8 valid_table[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];



void My_Init_Camera(void);

void UART_Image(void);
void UART_Thresholding_Image(void);
void UART_Thresholding_Value(void);
void UART_Inverse_Perspective(void);
void UART_Classification(void);

void Set_Thresholding_Value(uint8 val);
void Set_CameraAlphaUpOrDown(uint8 val);
void Set_CameraThetaDown(uint8 val);
void Set_RatioOfMaxDisToHG(uint8 val);
void Set_RatioOfPixelToHG(uint8 val);
//uint8 Classification_16(void);
uint8 Classification_25(void);
uint8 Classification_Classic(void);
uint8 Classification_Classic25(void);
void Check_Classification(uint8 classification_Result_tmp, uint8 check_counter);
void GetBinThreshold_OSTU(void);
void GetBinThreshold_OSTU_Inverse(void);




void Get_Cutted_Image(void);
void Get_Thresholding_Value(void);
void Get_Inverse_Perspective_Table(void);
void Get_Thresholding_Image(void);
void Get_Inverse_Perspective_Image(void);

float Filter(uint8 ID,float value,float ratio);


void Set_Search_Range(uint8 row_start,uint8 row_lines,uint8 col_start,uint8 col_lines);
#define ROW 0
#define BEGIN 0
#define LINES 1
#define COL 1




#endif
