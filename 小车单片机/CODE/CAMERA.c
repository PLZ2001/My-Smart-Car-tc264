#include "headfile.h"
#include "CAMERA.h"
#include "fastlz.h"//压缩算法
#include <stdlib.h>
#include "OLED.h"
#include "TIME.h"
#include "STEERING.h"
#include "SEARCH.h"
#include "MOTOR_CTL.h"
//摄像头高度约为19.5cm

//需要串口通信输出去，但不用传过来的变量
//IFX_ALIGN(4) uint8 mt9v03x_image_cutted[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];//裁剪后的原始图像
uint8 classification_Result;
uint8 classification_Result_2nd;
uint8 classification_Result_1;
uint8 classification_Result_1_2nd;
uint8 classification_Result_2;
uint8 classification_Result_2_2nd;

//需要串口通信传过来的变量（必须配以执行变量更新的函数）
uint8 thresholding_Value = 128; //对应的更新函数为：void Set_Thresholding_Value(uint8 val);
float cameraAlphaUpOrDown = 40.0f * 3.1415926 / 2 / 180;//无需校正
float cameraThetaDown = 26.89191f * 3.1415926 / 180;//需要校正
float ratioOfMaxDisToHG = 5.915322f;//仅影响显示距离
float ratioOfPixelToHG = 0.076f;//仅影响分辨率


//不需要传输的其他变量
IFX_ALIGN(4) uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];//原始图像
//IFX_ALIGN(4) uint8 mt9v03x_image_cutted_thresholding[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];
IFX_ALIGN(4) uint8 mt9v03x_image_cutted_thresholding_inversePerspective[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
int width_Inverse_Perspective;
int height_Inverse_Perspective;

uint8 flag_For_Right_Circle = 0;
uint8 flag_For_Left_Circle = 0;
uint8 flag_For_Right_T = 0;
uint8 flag_For_Left_T = 0;

//float BayesTable_16[13][CLASS_NUM] = {{-6.212,-9.319,-24.312,-25.426,-3.056,-29.253},
//        {25.361,26.676,57.032,54.606,25.934,60.921},
//        {30.833,28.652,55.679,57.031,30.260,59.570},
//        {-11.965,-9.801,-27.646,-27.080,-5.181,-31.199},
//        {-7.657,-8.620,-9.958,-20.068,1.053,-2.612},
//        {20.082,7.303,33.639,33.028,19.800,30.603},
//        {-7.004,11.375,13.713,21.002,4.998,17.201},
//        {-3.465,-3.579,-12.959,-6.872,5.111,0.053},
//        {155.160,54.548,219.792,151.069,241.736,245.156},
//        {52.537,147.741,143.563,208.364,230.583,233.707},
//        {138.356,156.841,144.860,165.326,99.307,119.594},
//        {122.261,105.980,128.375,112.617,70.663,89.646},
//        {-189.763,-187.540,-328.932,-331.373,-322.336,-380.283}};
float BayesTable_25[17][CLASS_NUM] = {{1.685,-3.593,1.532,-4.908,9.487,-2.020},
        {13.395,16.396,12.648,16.239,13.193,9.854},
        {9.275,9.163,63.356,62.429,7.743,66.090},
        {8.719,7.773,6.405,4.235,5.764,0.084},
        {2.974,7.324,1.222,6.714,13.834,3.711},
        {6.857,10.178,10.562,14.006,19.157,12.922},
        {-11.146,-14.156,-23.473,-26.963,-38.984,-28.164},
        {68.947,68.188,103.941,103.082,95.119,113.900},
        {-17.427,-15.530,-30.721,-25.538,-39.677,-30.052},
        {8.645,6.568,13.853,9.487,18.106,12.077},
        {34.226,-8.819,57.349,10.286,87.398,98.488},
        {-134.646,-130.190,-46.101,-41.610,-33.127,-37.576},
        {-1.891,35.422,10.037,56.403,84.903,96.292},
        {28.944,17.733,56.317,47.125,43.668,44.197},
        {621.139,621.478,643.436,630.584,554.176,585.571},
        {-8.681,3.995,19.472,31.779,22.297,20.645},
        {-254.031,-254.603,-427.897,-419.047,-383.225,-467.807}};

char *class_Name_Group[CLASS_NUM+10] = {"0左弯", "1右弯", "2左环岛", "3右环岛", "4三岔路口", "5十字路口","6直道","7靠左（临时使用）","8靠右（临时使用）", "9未知","10左直线","11右直线","12左丁字","13右丁字","14T字","15长直道"};

float arg_Classification_16[16];
float arg_Classification_25[25];
float arg_Classification_36[36];
uint8 arg_Classification_36_Table[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
uint8 arg_Classification_36_Table_1[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
uint8 arg_Classification_36_Table_2[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];


uint8 fuzzy_Image_25[5][5];
uint8 fuzzy_Image_36[6][6];
float fuzzy_thresholdingValue_25 = 0.60;
//float fuzzy_thresholdingValue_36 = 0.40;
float fuzzy_thresholdingValue_36 = 0.60;

//1表示：如果这里有道路，就会加分
//-1表示：如果这里有道路，就会扣分
//0表示：不用管
float  ModelTable_25[4][5][5]={{{ 1, 0, 0, 0, 1},
                                { 1, 1, 0, 1, 1},
                                { 0, 1, 1, 1, 0},
                                { 0, 1, 1, 1, 0},
                                { 0, 0, 0, 0, 0}}, {{ 0, 0, 1, 0, 0},
                                                    { 0, 0, 1, 0, 0},
                                                    { 0, 1, 1, 1, 0},
                                                    { 0, 1, 1, 1, 0},
                                                    { 0, 0, 0, 0, 0}}, {{ 0, 0, 1, 0, 0},
                                                                        { 0, 0, 1, 0, 0},
                                                                        { 0,-1, 1, 1, 0},
                                                                        { 0,-1, 1, 1, 0},
                                                                        { 0, 0, 0, 0, 0}}, {{ 0, 0, 1, 0, 0},
                                                                                            { 0, 0, 1, 0, 0},
                                                                                            { 0, 1, 1,-1, 0},
                                                                                            { 0, 1, 1,-1, 0},
                                                                                            { 0, 0, 0, 0, 0}}};// 三岔路口、十字路口、右环岛、左环岛
//表示标准的道路得分情况
uint8 ModelTable_25_Score[4] = {9,8,6,6};//{11,9,9,9};

//1表示：如果这里有道路，就会加分
//-1表示：如果这里有道路，就会扣分
//0表示：不用管
//float ModelTable_36[CLASS_NUM_NEW][6][6]={{{ 1, 0,-1,-1, 0, 1},
//                                           { 1, 1,-1,-1, 1, 1},
//                                           { 0, 1,-1,-1, 1, 0},
//                                           { 0, 1, 1, 1, 1, 0},
//                                           { 0, 0, 1, 1, 0, 0},
//                                           { 0, 0, 0, 0, 0, 0}}, {{ 0,-1, 1, 1,-1, 0},
//                                                                  { 0,-1, 1, 1,-1, 0},
//                                                                  { 0, 1, 1, 1, 1, 0},
//                                                                  { 0, 1, 1, 1, 1, 0},
//                                                                  { 0, 0, 1, 1, 0, 0},
//                                                                  { 0, 0, 0, 0, 0, 0}}, {{-1,-8, 1, 1,-1,-1},
//                                                                                         {-1,-8, 1, 1, 1, 1},
//                                                                                         { 0,-8, 1, 1, 1, 0},
//                                                                                         { 0,-8, 1, 1, 1, 0},
//                                                                                         { 0, 0, 1, 1, 0, 0},
//                                                                                         { 0, 0, 0, 0, 0, 0}}, {{-1,-1, 1, 1,-8,-1},
//                                                                                                                { 1, 1, 1, 1,-8,-1},
//                                                                                                                { 0, 1, 1, 1,-8, 0},
//                                                                                                                { 0, 1, 1, 1,-8, 0},
//                                                                                                                { 0, 0, 1, 1, 0, 0},
//                                                                                                                { 0, 0, 0, 0, 0, 0}},  {{ 0,-1, 1, 1,-1, 0},
//                                                                                                                                        { 0,-1, 1, 1,-1, 0},
//                                                                                                                                        { 0,-2, 1, 1,-1, 0},
//                                                                                                                                        { 0,-3, 1, 1, 1, 0},
//                                                                                                                                        { 0, 0, 1, 1, 0, 0},
//                                                                                                                                        { 0, 0, 0, 0, 0, 0}},  {{ 0,-1, 1, 1,-1, 0},
//                                                                                                                                                                { 0,-1, 1, 1,-1, 0},
//                                                                                                                                                                { 0,-1, 1, 1,-2, 0},
//                                                                                                                                                                { 0, 1, 1, 1,-3, 0},
//                                                                                                                                                                { 0, 0, 1, 1, 0, 0},
//                                                                                                                                                                { 0, 0, 0, 0, 0, 0}},  {{ 0,-1,-3,-3,-1, 0},
//                                                                                                                                                                                        { 0,-1,-3,-3,-1, 0},
//                                                                                                                                                                                        { 0, 1, 1, 1, 1, 0},
//                                                                                                                                                                                        { 0, 1, 1, 1, 1, 0},
//                                                                                                                                                                                        { 0, 0, 1, 1, 0, 0},
//                                                                                                                                                                                        { 0, 0, 0, 0, 0, 0}}};

float ModelTable_36[CLASS_NUM_NEW][6][6]={{{ 1,1,-20,-20,1, 1},
                                           { 1, 1, 0, 0, 1, 1},
                                           { 1, 1, 1, 1, 1, 1},
                                           { 1, 1, 1, 1, 1, 1},
                                           { 0, 1, 1, 1, 1, 0},
                                           { 0, 0, 1, 1, 0, 0}}, {{-10,0, 1, 1,0,-10},
                                                                  {-2, 0, 1, 1, 0,-2},
                                                                  { 1, 1, 1, 1, 1, 1},
                                                                  { 1, 1, 1, 1, 1, 1},
                                                                  { 1, 1, 1, 1, 1, 1},
                                                                  { 0, 0, 1, 1, 0, 0}}, {{-10,0, 1, 1,0,-10},
                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                         {-10,0, 1, 1, 1, 0},
                                                                                         {-10,0, 1, 1, 0, 0}}, {{-10,0, 1, 1, 0,-10},
                                                                                                                { 1, 1, 1, 1, 0,-10},
                                                                                                                { 1, 1, 1, 1, 0,-10},
                                                                                                                { 1, 1, 1, 1, 0,-10},
                                                                                                                { 0, 1, 1, 1, 0,-10},
                                                                                                                { 0, 0, 1, 1, 0,-10}},  {{-10,0, 1, 1, 0,-10},
                                                                                                                                         {-10,0, 1, 1,-2,-10},
                                                                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                                                                         {-10,0, 1, 1, 1, 1},
                                                                                                                                         {-10,0, 1, 1, 0, 0}},  {{-10, 0, 1, 1,0,-10},
                                                                                                                                                                 {-10,-2, 1, 1,0,-10},
                                                                                                                                                                 {  1, 1, 1, 1,0,-10},
                                                                                                                                                                 {  1, 1, 1, 1,0,-10},
                                                                                                                                                                 {  1, 1, 1, 1,0,-10},
                                                                                                                                                                 {  0, 0, 1, 1,0,-10}},  {{  0,  0,  0,  0,  0,  0},
                                                                                                                                                                                          {-10,-10,-10,-10,-10,-10},
                                                                                                                                                                                          {  0,  0,  0,  0,  0,  0},
                                                                                                                                                                                          {  1,  1,  1,  1,  1,  1},
                                                                                                                                                                                          {  1,  1,  1,  1,  1,  1},
                                                                                                                                                                                          {  0,  1,  1,  1,  1,  0}}};

// 三岔路口、十字路口、右环岛、左环岛、右丁字、左丁字、T字路口
//表示标准的道路得分情况
//uint8 ModelTable_36_Score[CLASS_NUM_NEW] = {12,14,12,12,11,11,10};
//float ModelTable_36_Score_Required[CLASS_NUM_NEW] = {0.6,0.8,0.6,0.6,0.6,0.6,0.6};
uint8 ModelTable_36_Score[CLASS_NUM_NEW] = {22,24,19,19,18,18,16};
float ModelTable_36_Score_Required[CLASS_NUM_NEW] = {-0.1,100.0/*0.15*/,0.2,0.2,0.15,0.15,-100.0};

float score[CLASS_NUM_NEW] = {0};
float max_Score = -72;

uint8 Inverse_Perspective_Table_Row[height_Inverse_Perspective_Max];
uint8 Inverse_Perspective_Table_Col[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];

uint8 Thresholding_Value_Init_Flag = 0;//表示是否初始化了二值化阈值


uint8 Search_Range[2][2]={{0,height_Inverse_Perspective_Max},{0,width_Inverse_Perspective_Max}};
//用来描述扫描的范围：{{起始行,扫描多少行},{起始列,扫描多少列}}

uint8 Long_Straight_Flag = 0;

int invalid_lines = 0;//指示下面无效的行数

uint8 Circle_EN = 1;

void My_Init_Camera(void)
{
    mt9v03x_init();
}

void UART_Image(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x01);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
//    uart_putbuff(DEBUG_UART, *mt9v03x_image_cutted, X_WIDTH_CAMERA*Y_WIDTH_CAMERA);  //发送压缩后的图像
    uart_putbuff(DEBUG_UART, *mt9v03x_image, MT9V03X_H*MT9V03X_W);  //发送原始图像
    //uart_putbuff(DEBUG_UART, *mt9v03x_image_cutted_compressed, compressed_Size);  //发送压缩后的图像
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x01);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void UART_Thresholding_Image(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x01);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uint32 len = MT9V03X_H*MT9V03X_W/8; //要求X_WIDTH_CAMERA*Y_WIDTH_CAMERA刚好是8的倍数
    uint8 *buff = *mt9v03x_image;
    while(len)
    {
        uint8 dat = ((*buff>thresholding_Value)<<7)
                | ((*(buff+1)>thresholding_Value)<<6)
                | ((*(buff+2)>thresholding_Value)<<5)
                | ((*(buff+3)>thresholding_Value)<<4)
                | ((*(buff+4)>thresholding_Value)<<3)
                | ((*(buff+5)>thresholding_Value)<<2)
                | ((*(buff+6)>thresholding_Value)<<1)
                | ((*(buff+7)>thresholding_Value));
        uart_putchar(DEBUG_UART, dat);
        len--;
        buff+=8;
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x01);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void UART_Thresholding_Value(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x02);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, thresholding_Value);  //发送二值化阈值
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x02);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void UART_Inverse_Perspective(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x03);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, (uint8)round((cameraAlphaUpOrDown*2*180/3.1415926 - 0)/(90-0)*255));
    uart_putchar(DEBUG_UART, (uint8)round((cameraThetaDown*180/3.1415926 - 0)/(90-0)*255));
    uart_putchar(DEBUG_UART, (uint8)round((ratioOfMaxDisToHG - 0)/(15-0)*255));
    uart_putchar(DEBUG_UART, (uint8)round((ratioOfPixelToHG - 0)/(0.1-0)*255));
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x03);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void UART_Classification(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x08);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, classification_Result);
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x08);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void Set_Thresholding_Value(uint8 val)
{
    thresholding_Value = val;
}

void Set_CameraAlphaUpOrDown(uint8 val)
{
    cameraAlphaUpOrDown = (((float)val)/255*(90-0)+0) * 3.1415926 / 2 / 180;
}
void Set_CameraThetaDown(uint8 val)
{
    cameraThetaDown = (((float)val)/255*(90-0)+0) * 3.1415926 / 180;
}
void Set_RatioOfMaxDisToHG(uint8 val)
{
    ratioOfMaxDisToHG = (((float)val)/255*(15-0)+0);
}
void Set_RatioOfPixelToHG(uint8 val)
{
    ratioOfPixelToHG = (((float)val)/255*(0.1-0)+0);
}

/*//压缩
void Get_Cutted_Image(void)
{
    uint8 div_h, div_w;
    uint32 temp_h = 0;
    uint32 temp_w = 0;
    uint32 row_start = 0;
    uint32 lin_start = 0;

    div_h = MT9V03X_H/Y_WIDTH_CAMERA;
    div_w = MT9V03X_W/X_WIDTH_CAMERA;

    //从中心取图像
    if(Y_WIDTH_CAMERA * div_h != MT9V03X_H)
    {
        row_start = (MT9V03X_H - Y_WIDTH_CAMERA * div_h)/2;
        temp_h = row_start;
    }
    if(X_WIDTH_CAMERA * div_w != MT9V03X_W)
    {
        lin_start = (MT9V03X_W - X_WIDTH_CAMERA * div_w)/2;
    }
    for(int i = 0; i < Y_WIDTH_CAMERA; i++)
    {
        temp_w = lin_start;
        for(int j = 0; j < X_WIDTH_CAMERA; j++)
        {
            mt9v03x_image_cutted[i][j] = mt9v03x_image[temp_h][temp_w];
            temp_w += div_w;
        }
        temp_h += div_h;
    }
}*/

//真正的裁剪
//void Get_Cutted_Image(void)
//{
//    int origin_i=0,origin_j=0;
//    origin_i = (MT9V03X_H - Y_WIDTH_CAMERA)/2;
//    origin_j = (MT9V03X_W - X_WIDTH_CAMERA)/2;
//    for(int i = 0; i < Y_WIDTH_CAMERA; i++)
//    {
//        origin_j = (MT9V03X_W - X_WIDTH_CAMERA)/2;
//        for(int j = 0; j < X_WIDTH_CAMERA; j++)
//        {
//            mt9v03x_image_cutted[i][j] = mt9v03x_image[origin_i][origin_j];
//            origin_j++;
//        }
////        origin_i+=2;
//        origin_i+=1;
//    }
//    int origin_i=0,origin_j=0;
//    origin_i = 40;
//    origin_j = 0;
//    for(int i = 0; i < Y_WIDTH_CAMERA; i++)
//    {
//        origin_i = 40 + i*2;
//        origin_j = 0;
//        for(int j = 0; j < X_WIDTH_CAMERA; j++)
//        {
//            origin_j = j;
//            mt9v03x_image_cutted[i][j] = mt9v03x_image[origin_i][origin_j];
//        }
//    }
//}



//曾经的分4类
//void Get_Thresholding_Image(void)
//{
//    //Kmeans法更新二值化阈值
//    float m[KMEANS_K][3] = {{51,0,0},{102,0,0},{153,0,0},{204,0,0}};//第一列存最终分类结果，第二列存每种分类的样本数，第三列存中间变量
//    float maxtimes = 2;
//    for (int time = 0;time<maxtimes;time++)
//    {
//        for (int i = 0;i<KMEANS_K;i++)
//        {
//            m[i][1] = 0;
//        }
//        for (int i = 0;i<Y_WIDTH_CAMERA;i++)
//        {
//            for (int j = 0;j<X_WIDTH_CAMERA;j++)
//            {
//                float min_distance = fabs(mt9v03x_image_cutted[i][j]-m[0][0]);
//                int min_distance_class = 0;
//                for (int k=1;k<KMEANS_K;k++)
//                {
//                    if (fabs(mt9v03x_image_cutted[i][j]-m[k][0]) < min_distance)
//                    {
//                        min_distance = fabs(mt9v03x_image_cutted[i][j]-m[k][0]);
//                        min_distance_class = k;
//                    }
//                }
//                m[min_distance_class][1] = m[min_distance_class][1]+1;
//                m[min_distance_class][2] = (m[min_distance_class][1]-1)/m[min_distance_class][1]*m[min_distance_class][2] + mt9v03x_image_cutted[i][j]/m[min_distance_class][1];
//            }
//        }
//        for (int i = 0;i<KMEANS_K;i++)
//        {
//            m[i][0] = m[i][2];
//        }
//    }
//    float max_cnt[2] = {-1,-1};
//    int max_cnt_class[2] = {0,0};
//    for (int i = 0;i<KMEANS_K;i++)
//    {
//        if (m[i][1]>max_cnt[0] && m[i][0]<GOD_LIGHT)
//        {
//            max_cnt[1] = max_cnt[0];
//            max_cnt_class[1] = max_cnt_class[0];
//            max_cnt[0] = m[i][1];
//            max_cnt_class[0] = i;
//        }
//        else if (m[i][1]>max_cnt[1] && m[i][0]<GOD_LIGHT)
//        {
//            max_cnt[1] = m[i][1];
//            max_cnt_class[1] = i;
//        }
//    }
//    thresholding_Value = (uint8)(0.5*(m[max_cnt_class[0]][0]+m[max_cnt_class[1]][0]));
//
//    for(int j = 0; j < Y_WIDTH_CAMERA; j++)
//    {
//        for(int i = 0; i < X_WIDTH_CAMERA; i++)
//        {
//            mt9v03x_image_cutted_thresholding[j][i] = mt9v03x_image_cutted[j][i]>thresholding_Value;
//        }
//    }
//}

//void Get_Thresholding_Image(void)
//{
//    //Kmeans法更新二值化阈值
//    float m[KMEANS_K][3] = {{85,0,0},{170,0,0}};//第一列存最终分类结果，第二列存每种分类的样本数，第三列存中间变量
//    float maxtimes = 2;
//    for (int time = 0;time<maxtimes;time++)
//    {
//        for (int i = 0;i<KMEANS_K;i++)
//        {
//            m[i][1] = 0;
//        }
//        for (int i = 0;i<Y_WIDTH_CAMERA;i++)
//        {
//            for (int j = 0;j<X_WIDTH_CAMERA;j++)
//            {
//                float min_distance = fabs(mt9v03x_image_cutted[i][j]-m[0][0]);
//                int min_distance_class = 0;
//                for (int k=1;k<KMEANS_K;k++)
//                {
//                    if (fabs(mt9v03x_image_cutted[i][j]-m[k][0]) < min_distance)
//                    {
//                        min_distance = fabs(mt9v03x_image_cutted[i][j]-m[k][0]);
//                        min_distance_class = k;
//                    }
//                }
//                m[min_distance_class][1] = m[min_distance_class][1]+1;
//                m[min_distance_class][2] = (m[min_distance_class][1]-1)/m[min_distance_class][1]*m[min_distance_class][2] + mt9v03x_image_cutted[i][j]/m[min_distance_class][1];
//            }
//        }
//        for (int i = 0;i<KMEANS_K;i++)
//        {
//            m[i][0] = m[i][2];
//        }
//    }
//    thresholding_Value = (uint8)(0.5*(m[0][0]+m[1][0]));
//
//    for(int j = 0; j < Y_WIDTH_CAMERA; j++)
//    {
//        for(int i = 0; i < X_WIDTH_CAMERA; i++)
//        {
//            mt9v03x_image_cutted_thresholding[j][i] = mt9v03x_image_cutted[j][i]>thresholding_Value;
//        }
//    }
//}

void Get_Thresholding_Value(void)
{
    float m[KMEANS_K][3] = {{85,0,0},{170,0,0}};//第一列存最终分类结果，第二列存每种分类的样本数，第三列存中间变量
    float maxtimes = 2;
    for (int time = 0;time<maxtimes;time++)
    {
       for (int i = 0;i<KMEANS_K;i++)
       {
           m[i][1] = 0;
       }
       for (int i = 0;i<Y_WIDTH_CAMERA;i++)
       {
           for (int j = 0;j<X_WIDTH_CAMERA;j++)
           {
               float min_distance = fabs(mt9v03x_image[i][j]-m[0][0]);
               int min_distance_class = 0;
               for (int k=1;k<KMEANS_K;k++)
               {
                   if (fabs(mt9v03x_image[i][j]-m[k][0]) < min_distance)
                   {
                       min_distance = fabs(mt9v03x_image[i][j]-m[k][0]);
                       min_distance_class = k;
                   }
               }
               m[min_distance_class][1] = m[min_distance_class][1]+1;
               m[min_distance_class][2] = (m[min_distance_class][1]-1)/m[min_distance_class][1]*m[min_distance_class][2] + mt9v03x_image[i][j]/m[min_distance_class][1];
           }
       }
       for (int i = 0;i<KMEANS_K;i++)
       {
           m[i][0] = m[i][2];
       }
    }
    thresholding_Value = (uint8)(0.5*(m[0][0]+m[1][0]));//二值化阈值更新结果
}

float Filter(uint8 ID,float value,float ratio)
{
    switch(ID)
    {
        case 1:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 2:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 3:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 4:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 5:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 6:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 7:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        case 8:
        {
            static uint8 flag=0;
            static float last_value=0;
            float new_value;
            if (flag==0)
            {
                last_value=value;
                flag=1;
                return value;
            }
            else if(flag==1)
            {
                new_value=ratio*value+(1-ratio)*last_value;
                last_value = new_value;
                return new_value;
            }
            break;
        }
        default:
        {
            return value;
            break;
        }
    }
}


void Get_Thresholding_Image(void)
{
//    //Kmeans法更新二值化阈值
    //if (Read_Timer(3) > time_up[3] && steering_Target>=-5 && steering_Target>=5) {
//    if (steering_Target>=-5 && steering_Target<=5) {
//    if ((classification_Result==9||classification_Result==6)&&(steering_Target>=-5 && steering_Target<=5)&&(speed_Status!=Highest)) {
//    if ((steering_Target>=-3 && steering_Target<=3)&&(speed_Status==Low)) {
    if (classification_Result!=2&&classification_Result!=3&&speed_Status!=Highest){
        Reset_Timer(3);
//        Get_Thresholding_Value();
        GetBinThreshold_OSTU();//大津法二值化
        thresholding_Value = Filter(1,thresholding_Value,1);
        Start_Timer(3);
    }

//    for(int j = 0; j < Y_WIDTH_CAMERA; j++)
//    {
//        for(int i = 0; i < X_WIDTH_CAMERA; i++)
//        {
//            mt9v03x_image_cutted_thresholding[j][i] = mt9v03x_image_cutted[j][i]>thresholding_Value;
//        }
//    }
}


void Get_Inverse_Perspective_Table(void)
{
    int ratio=1;
    width_Inverse_Perspective = (int)round(2 * (X_WIDTH_CAMERA*1.0) / (ratio*Y_WIDTH_CAMERA*1.0) * tan(cameraAlphaUpOrDown) / cos(cameraThetaDown) * ratioOfMaxDisToHG / ratioOfPixelToHG);
    height_Inverse_Perspective = (int)round(ratioOfMaxDisToHG / ratioOfPixelToHG);
    for (int j_Processed = 0;j_Processed<height_Inverse_Perspective;j_Processed++)
    {
        float y_Processed = (float)(height_Inverse_Perspective - 1 - j_Processed);
        float y = (tan(cameraThetaDown)-1.0/y_Processed/ratioOfPixelToHG)*ratio*(Y_WIDTH_CAMERA*1.0)/2.0/tan(cameraAlphaUpOrDown);
        int j = (int)round(-y + (ratio*Y_WIDTH_CAMERA*1.0-1)/2.0);


        if (j>=0 && j<ratio*Y_WIDTH_CAMERA*1.0)
        {
            Inverse_Perspective_Table_Row[j_Processed]=(uint8)(j/ratio);//存表
            for (int i_Processed = 0;i_Processed<width_Inverse_Perspective;i_Processed++)
            {
                float x_Processed = (float)i_Processed - ((float)width_Inverse_Perspective-1)/2.0;
                float x = x_Processed*(ratio*Y_WIDTH_CAMERA*1.0/2.0/tan(cameraAlphaUpOrDown)*sin(cameraThetaDown)-y*cos(cameraThetaDown))*ratioOfPixelToHG;
                int i = (int)round(x + (X_WIDTH_CAMERA*1.0-1)/2.0);
                if (i>=0 && i<X_WIDTH_CAMERA*1.0)
                {
                    Inverse_Perspective_Table_Col[j_Processed][i_Processed] = (uint8)i;//存表
                }
                else
                {
                    Inverse_Perspective_Table_Col[j_Processed][i_Processed] = 255;//非法
                }
            }
        }
        else
        {
            Inverse_Perspective_Table_Row[j_Processed]=255;//非法
            invalid_lines = height_Inverse_Perspective - j_Processed;
            height_Inverse_Perspective = j_Processed+1;
            return;
        }
    }
}


void Get_Inverse_Perspective_Image(void)
{
    for (int j_Processed = 0;j_Processed<height_Inverse_Perspective;j_Processed++)
    {
        for (int i_Processed = 0;i_Processed<width_Inverse_Perspective;i_Processed++)
        {
            if (Inverse_Perspective_Table_Row[j_Processed]!=255 && Inverse_Perspective_Table_Col[j_Processed][i_Processed]!=255)
            {
                mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = (mt9v03x_image[Inverse_Perspective_Table_Row[j_Processed]][Inverse_Perspective_Table_Col[j_Processed][i_Processed]])>thresholding_Value;
            }
            else
            {
                mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = 255;
            }
        }
    }
}
//
//void Get_Inverse_Perspective_Image(void)
//{
//    int ratio=2;
//    width_Inverse_Perspective = (int)round(2 * (X_WIDTH_CAMERA*1.0) / (ratio*Y_WIDTH_CAMERA*1.0) * tan(cameraAlphaUpOrDown) / cos(cameraThetaDown) * ratioOfMaxDisToHG / ratioOfPixelToHG);
//    height_Inverse_Perspective = (int)round(ratioOfMaxDisToHG / ratioOfPixelToHG);
//
//    for (int j_Processed = 0;j_Processed<height_Inverse_Perspective;j_Processed++)
//    {
//        float y_Processed = (float)(height_Inverse_Perspective - 1 - j_Processed);
//        float y = (tan(cameraThetaDown)-1.0/y_Processed/ratioOfPixelToHG)*ratio*(Y_WIDTH_CAMERA*1.0)/2.0/tan(cameraAlphaUpOrDown);
//        int j = (int)round(-y + (ratio*Y_WIDTH_CAMERA*1.0-1)/2.0);
//        if (j>=0 && j<ratio*Y_WIDTH_CAMERA*1.0)
//        {
//            for (int i_Processed = 0;i_Processed<width_Inverse_Perspective;i_Processed++)
//            {
//                float x_Processed = (float)i_Processed - ((float)width_Inverse_Perspective-1)/2.0;
//                float x = x_Processed*(ratio*Y_WIDTH_CAMERA*1.0/2.0/tan(cameraAlphaUpOrDown)*sin(cameraThetaDown)-y*cos(cameraThetaDown))*ratioOfPixelToHG;
//                int i = (int)round(x + (X_WIDTH_CAMERA*1.0-1)/2.0);
//                if (i>=0 && i<X_WIDTH_CAMERA*1.0)
//                {
//                    mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = mt9v03x_image_cutted_thresholding[j/ratio][i];
//                }
//                else
//                {
//                    mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = 255;
//                }
//            }
//        }
//        else
//        {
//            for (int i_Processed = 1;i_Processed<width_Inverse_Perspective;i_Processed++)
//            {
//                mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = 255;
//            }
//        }
//    }
//}
//
//void Get16(float* arg)
//{
//    int col_edge[5],row_edge[5];
//    col_edge[0] = 0;
//    col_edge[1] = width_Inverse_Perspective/4;
//    col_edge[2] = 2*col_edge[1];
//    col_edge[3] = 3*col_edge[1];
//    col_edge[4] = width_Inverse_Perspective-1;
//    row_edge[0] = 0;
//    row_edge[1] = height_Inverse_Perspective/4;
//    row_edge[2] = 2*row_edge[1];
//    row_edge[3] = 3*row_edge[1];
//    row_edge[4] = height_Inverse_Perspective-1;
//
//    int white_cnt[16] = {0};
//    int black_cnt[16] = {0};
//    for (int i = 0;i<height_Inverse_Perspective;i++)
//    {
//        for (int j = 0;j<width_Inverse_Perspective;j++)
//        {
//            white_cnt[4*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[4*(i/row_edge[1]) + j/col_edge[1]];
//            black_cnt[4*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[4*(i/row_edge[1]) + j/col_edge[1]];
//        }
//    }
//    for (int i = 0;i<16;i++)
//    {
//        if ((white_cnt[i]+black_cnt[i]) == 0)
//        {
//           arg[i] = 0;
//        }
//        else
//        {
//           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
//        }
//    }
//}
//
//uint8 Classification_16(void)
//{
//    //float arg_Classification_16[16];
//    Get16(arg_Classification_16);
//    float classification_Data[CLASS_NUM] = {0};
//    float classification_Data_max1=0;
//    uint8 classification_Result_temp1=0;
//    float classification_Data_max2=0;
//    uint8 classification_Result_temp2=0;
//    for (uint8 i = 0;i<CLASS_NUM;i++)
//    {
//        classification_Data[i] = arg_Classification_16[0]*BayesTable_16[0][i]+
//                                arg_Classification_16[1]*BayesTable_16[1][i]+
//                                arg_Classification_16[2]*BayesTable_16[2][i]+
//                                arg_Classification_16[3]*BayesTable_16[3][i]+
//                                arg_Classification_16[4]*BayesTable_16[4][i]+
//                                arg_Classification_16[5]*BayesTable_16[5][i]+
//                                arg_Classification_16[6]*BayesTable_16[6][i]+
//                                arg_Classification_16[7]*BayesTable_16[7][i]+
//                                arg_Classification_16[9]*BayesTable_16[8][i]+
//                                arg_Classification_16[10]*BayesTable_16[9][i]+
//                                arg_Classification_16[13]*BayesTable_16[10][i]+
//                                arg_Classification_16[14]*BayesTable_16[11][i]+
//                                BayesTable_16[12][i];
//
//        if (classification_Data[i]>classification_Data_max1)
//        {
//            classification_Data_max2 = classification_Data_max1;
//            classification_Result_temp2 = classification_Result_temp1;
//            classification_Data_max1 = classification_Data[i];
//            classification_Result_temp1 = i;
//        }
//        else if (classification_Data[i]>classification_Data_max2)
//        {
//            classification_Data_max2 = classification_Data[i];
//            classification_Result_temp2 = i;
//        }
//    }
//    if ((classification_Data_max1-classification_Data_max2) > CLASSIFICATION_16_VALID)
//    {
//        return classification_Result_temp1;
//    }
//    else
//    {
//        return 9;//9代表未知
//    }
//}
//
//void Get25(float* arg)
//{
//    int col_edge[6],row_edge[6];
//    col_edge[0] = 0;
//    col_edge[1] = width_Inverse_Perspective/5;
//    col_edge[2] = 2*col_edge[1];
//    col_edge[3] = 3*col_edge[1];
//    col_edge[4] = 4*col_edge[1];
//    col_edge[5] = width_Inverse_Perspective-1;
//    row_edge[0] = 0;
//    row_edge[1] = height_Inverse_Perspective/5;
//    row_edge[2] = 2*row_edge[1];
//    row_edge[3] = 3*row_edge[1];
//    row_edge[4] = 4*row_edge[1];
//    row_edge[5] = height_Inverse_Perspective-1;
//
//    int white_cnt[25] = {0};
//    int black_cnt[25] = {0};
//    for (int i = 0;i<height_Inverse_Perspective;i++)
//    {
//        for (int j = 0;j<width_Inverse_Perspective;j++)
//        {
//            white_cnt[5*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[5*(i/row_edge[1]) + j/col_edge[1]];
//            black_cnt[5*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[5*(i/row_edge[1]) + j/col_edge[1]];
//        }
//    }
//    for (int i = 0;i<25;i++)
//    {
//        if ((white_cnt[i]+black_cnt[i]) == 0)
//        {
//           arg[i] = 0;
//        }
//        else
//        {
//           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
//        }
//    }
//}
//
//void New_Get25(float* arg)
//{
//    float col_edge,row_edge;
//    col_edge = width_Inverse_Perspective/5.0f;
//    row_edge = height_Inverse_Perspective/5.0f;
//
//    int white_cnt[25] = {0};
//    int black_cnt[25] = {0};
//    for (int i = 0;i<height_Inverse_Perspective;i++)
//    {
//        for (int j = 0;j<width_Inverse_Perspective;j++)
//        {
//            white_cnt[(int)(5*(ceil((i+1)/row_edge)-1) + (ceil((j+1)/col_edge))-1)] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[(int)(5*(ceil((i+1)/row_edge)-1) + (ceil((j+1)/col_edge))-1)];
//            black_cnt[(int)(5*(ceil((i+1)/row_edge)-1) + (ceil((j+1)/col_edge))-1)] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[(int)(5*(ceil((i+1)/row_edge)-1) + (ceil((j+1)/col_edge))-1)];
//        }
//    }
//    for (int i = 0;i<25;i++)
//    {
//        if ((white_cnt[i]+black_cnt[i]) == 0)
//        {
//           arg[i] = 0;
//        }
//        else
//        {
//           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
//        }
//    }
//}

void New_Get36(float* arg)
{
    float col_edge,row_edge;
    col_edge = Search_Range[COL][LINES]/6.0f;
    row_edge = Search_Range[ROW][LINES]/6.0f;

    static uint8 flag = 1;
    if (flag == 1)
    {
        for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
        {
            for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
            {
                arg_Classification_36_Table[i][j] = (uint8)(6*(ceil((i-Search_Range[ROW][BEGIN]+1)/row_edge)-1) + (ceil((j-Search_Range[COL][BEGIN]+1)/col_edge))-1);
            }
        }
        flag = 0;
    }

    int white_cnt[36] = {0};
    int black_cnt[36] = {0};
    for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
    {
        for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
        {
            white_cnt[arg_Classification_36_Table[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[arg_Classification_36_Table[i][j]];
            black_cnt[arg_Classification_36_Table[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[arg_Classification_36_Table[i][j]];
        }
    }
    for (int i = 0;i<36;i++)
    {
        if ((white_cnt[i]+black_cnt[i]) == 0)
        {
           arg[i] = 0;
        }
        else
        {
           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
        }
    }
}

void New_Get36_1(float* arg)
{
    float col_edge,row_edge;
    col_edge = Search_Range[COL][LINES]/6.0f;
    row_edge = Search_Range[ROW][LINES]/6.0f;

    static uint8 flag = 1;
    if (flag == 1)
    {
        for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
        {
            for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
            {
                arg_Classification_36_Table_1[i][j] = (uint8)(6*(ceil((i-Search_Range[ROW][BEGIN]+1)/row_edge)-1) + (ceil((j-Search_Range[COL][BEGIN]+1)/col_edge))-1);
            }
        }
        flag = 0;
    }

    int white_cnt[36] = {0};
    int black_cnt[36] = {0};
    for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
    {
        for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
        {
            white_cnt[arg_Classification_36_Table_1[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[arg_Classification_36_Table_1[i][j]];
            black_cnt[arg_Classification_36_Table_1[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[arg_Classification_36_Table_1[i][j]];
        }
    }
    for (int i = 0;i<36;i++)
    {
        if ((white_cnt[i]+black_cnt[i]) == 0)
        {
           arg[i] = 0;
        }
        else
        {
           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
        }
    }
}

void New_Get36_2(float* arg)
{
    float col_edge,row_edge;
    col_edge = Search_Range[COL][LINES]/6.0f;
    row_edge = Search_Range[ROW][LINES]/6.0f;

    static uint8 flag = 1;
    if (flag == 1)
    {
        for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
        {
            for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
            {
                arg_Classification_36_Table_2[i][j] = (uint8)(6*(ceil((i-Search_Range[ROW][BEGIN]+1)/row_edge)-1) + (ceil((j-Search_Range[COL][BEGIN]+1)/col_edge))-1);
            }
        }
        flag = 0;
    }

    int white_cnt[36] = {0};
    int black_cnt[36] = {0};
    for (int i = Search_Range[ROW][BEGIN];i<Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES];i++)
    {
        for (int j = Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
        {
            white_cnt[arg_Classification_36_Table_2[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[arg_Classification_36_Table_2[i][j]];
            black_cnt[arg_Classification_36_Table_2[i][j]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[arg_Classification_36_Table_2[i][j]];
        }
    }
    for (int i = 0;i<36;i++)
    {
        if ((white_cnt[i]+black_cnt[i]) == 0)
        {
           arg[i] = 0;
        }
        else
        {
           arg[i] = white_cnt[i]*1.0f/(white_cnt[i]+black_cnt[i]);
        }
    }
}
//
//uint8 Classification_25(void)
//{
//    Get25(arg_Classification_25);
//    float classification_Data[CLASS_NUM] = {0};
//    float classification_Data_max1=0;
//    uint8 classification_Result_temp1=0;
//    float classification_Data_max2=0;
//    uint8 classification_Result_temp2=0;
//    for (uint8 i = 0;i<CLASS_NUM;i++)
//    {
//        classification_Data[i] = arg_Classification_25[0]*BayesTable_25[0][i]+
//                arg_Classification_25[1]*BayesTable_25[1][i]+
//                arg_Classification_25[2]*BayesTable_25[2][i]+
//                arg_Classification_25[3]*BayesTable_25[3][i]+
//                arg_Classification_25[4]*BayesTable_25[4][i]+
//                arg_Classification_25[5]*BayesTable_25[5][i]+
//                arg_Classification_25[6]*BayesTable_25[6][i]+
//                arg_Classification_25[7]*BayesTable_25[7][i]+
//                arg_Classification_25[8]*BayesTable_25[8][i]+
//                arg_Classification_25[9]*BayesTable_25[9][i]+
//                arg_Classification_25[11]*BayesTable_25[10][i]+
//                arg_Classification_25[12]*BayesTable_25[11][i]+
//                arg_Classification_25[13]*BayesTable_25[12][i]+
//                arg_Classification_25[16]*BayesTable_25[13][i]+
//                arg_Classification_25[17]*BayesTable_25[14][i]+
//                arg_Classification_25[18]*BayesTable_25[15][i]+
//                BayesTable_25[16][i];
//
//
//        if (classification_Data[i]>classification_Data_max1)
//        {
//            classification_Data_max2 = classification_Data_max1;
//            classification_Result_temp2 = classification_Result_temp1;
//            classification_Data_max1 = classification_Data[i];
//            classification_Result_temp1 = i;
//        }
//        else if (classification_Data[i]>classification_Data_max2)
//        {
//            classification_Data_max2 = classification_Data[i];
//            classification_Result_temp2 = i;
//        }
//    }
//    if ((classification_Data_max1-classification_Data_max2) > CLASSIFICATION_25_VALID)
//    {
//        return classification_Result_temp1;
//    }
//    else
//    {
//        return 9;//9代表未知
//    }
//}
//
//uint8 Classification_Classic(void)
//{
//    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
//    int Conv_Core[7][3][3] = {{{1,1,-1},{1,1,-1},{-1,-1,-1}},
//                              {{-1,1,1},{-1,1,1},{-1,-1,-1}},
//                              {{1,1,1},{1,1,-1},{1,-1,-1}},
//                              {{1,-1,-1},{1,-1,-1},{1,-1,-1}},
//                              {{-1,-1,1},{-1,-1,1},{-1,-1,1}},
//                              {{1,1,1},{1,1,-1},{1,-1,-1}},
//                              {{1,1,1},{-1,1,1},{-1,-1,1}}};
//    //十字路口
//    //Conv_Core(0).core = [1 1 -1
////                         1 1 -1
////                        -1 -1 -1];
//    //Conv_Core(1).core = [-1 1 1
////                         -1 1 1
////                        -1 -1 -1];
//    //右圆环
//    //Conv_Core(2).core = [1 1 1
////                         1 1 -1
////                         1 -1 -1];
//    //Conv_Core(3).core = [1 -1 -1
////                         1 -1 -1
////                         1 -1 -1];
//    //Conv_Core(4).core = [-1 -1 1
////                         -1 -1 1
////                         -1 -1 1];
//    //三岔路口
//    //Conv_Core(5).core = [1 1 1
////                         1 1 -1
////                         1 -1 -1];
//    //Conv_Core(6).core = [1 1 1
////                        -1 1 1
////                        -1 -1 1];
//    int Conv_Score_max[7] = {-9,-9,-9,-9,-9,-9,-9};
//    int Conv_Score_max_i[7] = {0,0,0,0,0,0,0};
//    int Conv_Score_max_j[7] = {0,0,0,0,0,0,0};
//    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
//    {
//        for (int j=1;j<width_Inverse_Perspective-1;j++)
//        {
//            // 对于每个中心点(i,j)，计算7类卷积值，分别取最大值保留
//            int Conv_Score[7] = {0,0,0,0,0,0,0};//存储7类卷积结果
//            int flag = 1;//卷积合不合法
//            for (int k=0;k<7;k++)
//            {
//                for (int ii=0;ii<3;ii++)
//                {
//                    for (int jj=0;jj<3;jj++)
//                    {
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[i-1+ii][j-1+jj] == 255)
//                        {
//                            flag = 0;
//                            break;
//                        }
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[i-1+ii][j-1+jj] == 0)
//                        {
//                            Conv_Score[k] = Conv_Score[k] +1*Conv_Core[k][ii][jj];
//                        }
//                        else
//                        {
//                            Conv_Score[k] = Conv_Score[k] +(-1)*Conv_Core[k][ii][jj];
//                        }
//                    }
//                    if (flag == 0)
//                    {
//                        break;
//                    }
//                }
//                if (flag == 0)
//                {
//                    break;
//                }
//            }
//            if (flag == 0)
//            {
//                continue;
//            }
//            //十字路口
//            //Conv_Core(0).core = [1 1 -1
//        //                         1 1 -1
//        //                        -1 -1 -1];
//            //Conv_Core(1).core = [-1 1 1
//        //                         -1 1 1
//        //                        -1 -1 -1];
//            //右圆环
//            //Conv_Core(2).core = [1 1 1
//        //                         1 1 -1
//        //                         1 -1 -1];
//            //Conv_Core(3).core = [1 -1 -1
//        //                         1 -1 -1
//        //                         1 -1 -1];
//            //Conv_Core(4).core = [-1 -1 1
//        //                         -1 -1 1
//        //                         -1 -1 1];
//            //三岔路口
//            //Conv_Core(5).core = [1 1 1
//        //                         1 1 -1
//        //                         1 -1 -1];
//            //Conv_Core(6).core = [1 1 1
//        //                        -1 1 1
//        //                        -1 -1 1];
//            //十字路口
//            if (Conv_Score[0] > Conv_Score_max[0] && j<width_Inverse_Perspective*0.6)
//            {
//                Conv_Score_max[0] = Conv_Score[0];
//                Conv_Score_max_i[0] = i;
//                Conv_Score_max_j[0] = j;
//            }
//            if (Conv_Score[1] > Conv_Score_max[1] && j>width_Inverse_Perspective*0.4)
//            {
//                Conv_Score_max[1] = Conv_Score[1];
//                Conv_Score_max_i[1] = i;
//                Conv_Score_max_j[1] = j;
//            }
//            //右圆环
//            if (Conv_Score[2] > Conv_Score_max[2] && i<height_Inverse_Perspective*0.5)
//            {
//                Conv_Score_max[2] = Conv_Score[2];
//                Conv_Score_max_i[2] = i;
//                Conv_Score_max_j[2] = j;
//            }
//            if (Conv_Score[3] > Conv_Score_max[3] && j<width_Inverse_Perspective*0.5)
//            {
//                Conv_Score_max[3] = Conv_Score[3];
//                Conv_Score_max_i[3] = i;
//                Conv_Score_max_j[3] = j;
//            }
//            if (Conv_Score[4] > Conv_Score_max[4] && j>width_Inverse_Perspective*0.5)
//            {
//                Conv_Score_max[4] = Conv_Score[4];
//                Conv_Score_max_i[4] = i;
//                Conv_Score_max_j[4] = j;
//            }
//            //三岔路口
//            if (Conv_Score[5] > Conv_Score_max[5] && j>width_Inverse_Perspective*0.6)
//            {
//                Conv_Score_max[5] = Conv_Score[5];
//                Conv_Score_max_i[5] = i;
//                Conv_Score_max_j[5] = j;
//            }
//            if (Conv_Score[6] > Conv_Score_max[6] && j<width_Inverse_Perspective*0.4)
//            {
//                Conv_Score_max[6] = Conv_Score[6];
//                Conv_Score_max_i[6] = i;
//                Conv_Score_max_j[6] = j;
//            }
//        }
//    }
//    //"2左环岛", "3右环岛", "4三岔路口", "5十字路口"
//    if (Conv_Score_max[0] >= 9 && Conv_Score_max[1] >= 9)
//    {
//        return 5;
//    }
//    else if (Conv_Score_max[2] >= 9 && Conv_Score_max[3] >= 9 && Conv_Score_max[4] >= 9)
//    //else if (Conv_Score_max[2] >= 9)
//    {
//        return 3;
//    }
//    else if (Conv_Score_max[5] >= 9 && Conv_Score_max[6] >= 9)
//    {
//        return 4;
//    }
//    else
//    {
//        return 9;//9未知
//    }
//
//}
//
//uint8 Classification_Classic25(void)
//{
//    New_Get25(arg_Classification_25);
//    float max_Score = -25;
//    float score = 0;
//    uint8 max_Socre_Result = 9;//9未知
//    for (int i = 0; i < 5; i++)
//    {
//        for (int j = 0; j < 5; j++)
//        {
//            fuzzy_Image_25[i][j] = arg_Classification_25[i*5+j]>fuzzy_thresholdingValue_25;
//        }
//    }
//    for (int k = 0; k < 4; k++)
//    {
//        score = 0;
//        for (int i = 0; i < 5; i++)
//        {
//            for (int j = 0; j < 5 ; j++)
//            {
//                score = score + arg_Classification_25[i*5+j]*ModelTable_25[k][i][j];
//            }
//        }
//        if (score > max_Score)
//        {
//            max_Score = score;
//            max_Socre_Result = k;
//        }
//    }
//
//    if (max_Socre_Result!=9 && max_Score>=ModelTable_25_Score[max_Socre_Result]*(0.1+fuzzy_thresholdingValue_25))
//    {
//        //"2左环岛", "3右环岛", "4三岔路口", "5十字路口"
//        switch(max_Socre_Result)
//        {
//            case 0:
//                return 4;
//            case 1:
//                return 5;
//            case 2:
//                return 3;
//            case 3:
//                return 2;
//            default:
//                return 9;//未知
//        }
//    }
//    else
//    {
//        return 9;//未知
//    }
//}

void Classification_Classic36(uint8 window_ID,uint8 *classification_Result_address,uint8 *classification_Result_2nd_address)
{
    if (window_ID==0)
    {
        New_Get36(arg_Classification_36);
    }
    else if (window_ID==1)
    {
        New_Get36_1(arg_Classification_36);
    }
    else if (window_ID==2)
    {
        New_Get36_2(arg_Classification_36);
    }
    uint8 max_Socre_Result = 9;//9未知
    uint8 second_Score_Result = 9;
    max_Score = -72;
    float second_Score = -72;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            fuzzy_Image_36[i][j] = arg_Classification_36[i*6+j]>fuzzy_thresholdingValue_36;
        }
    }
    for (int k = 0; k < CLASS_NUM_NEW; k++)
    {
        score[k] = 0;
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                score[k] = score[k] + arg_Classification_36[i*6+j]*ModelTable_36[k][i][j];
            }
        }
        if (k==2 || k==3)//额外的判断：环岛中间两竖列白色占比必须全部大于0.4，可以避免一些误识别
        {
            if (arg_Classification_36[2] > 0.15
                    && arg_Classification_36[3] > 0.15
                    && arg_Classification_36[8] > 0.15
                    && arg_Classification_36[9] > 0.15
                    && arg_Classification_36[14] > 0.15
                    && arg_Classification_36[15] > 0.15
                    && arg_Classification_36[20] > 0.15
                    && arg_Classification_36[21] > 0.15
                    && arg_Classification_36[26] > 0.15
                    && arg_Classification_36[27] > 0.15
                    && arg_Classification_36[32] > 0.15
                    && arg_Classification_36[33] > 0.15)
            {
                ;
            }
            else
            {
                score[k] = -72;
            }

            if (Circle_EN == 0)
            {
                score[k] = -72;
            }
        }
        if (k==1)//额外的判断：十字路口中间两竖列白色占比必须全部大于0.3，可以避免一些误识别
        {
            if (arg_Classification_36[2] > 0.15
                    && arg_Classification_36[3] > 0.15
                    && arg_Classification_36[8] > 0.15
                    && arg_Classification_36[9] > 0.15
                    && arg_Classification_36[14] > 0.15
                    && arg_Classification_36[15] > 0.15
                    && arg_Classification_36[20] > 0.15
                    && arg_Classification_36[21] > 0.15
                    && arg_Classification_36[26] > 0.15
                    && arg_Classification_36[27] > 0.15
                    && arg_Classification_36[32] > 0.15
                    && arg_Classification_36[33] > 0.15)
            {
                ;
            }
            else
            {
                score[k] = -72;
            }
        }
//        if (k==4||k==5)//额外的判断：左右丁字中间两竖列白色占比必须全部大于0.3，可以避免一些误识别
//        {
//            if (arg_Classification_36[2] > 0.3
//                    && arg_Classification_36[3] > 0.3
//                    && arg_Classification_36[8] > 0.3
//                    && arg_Classification_36[9] > 0.3
//                    && arg_Classification_36[14] > 0.3
//                    && arg_Classification_36[15] > 0.3
//                    && arg_Classification_36[20] > 0.3
//                    && arg_Classification_36[21] > 0.3
//                    && arg_Classification_36[26] > 0.3
//                    && arg_Classification_36[27] > 0.3
//                    && arg_Classification_36[32] > 0.3
//                    && arg_Classification_36[33] > 0.3)
//            {
//                ;
//            }
//            else
//            {
//                score[k] = -72;
//            }
//        }
        if ((score[k] - ModelTable_36_Score[k]*fuzzy_thresholdingValue_36)/(ModelTable_36_Score[k]*fuzzy_thresholdingValue_36) > max_Score)
        {
            second_Score = max_Score;
            second_Score_Result = max_Socre_Result;
            max_Score = (score[k] - ModelTable_36_Score[k]*fuzzy_thresholdingValue_36)/(ModelTable_36_Score[k]*fuzzy_thresholdingValue_36);
            max_Socre_Result = k;
        }
        else if((score[k] - ModelTable_36_Score[k]*fuzzy_thresholdingValue_36)/(ModelTable_36_Score[k]*fuzzy_thresholdingValue_36) > second_Score)
        {
            second_Score = (score[k] - ModelTable_36_Score[k]*fuzzy_thresholdingValue_36)/(ModelTable_36_Score[k]*fuzzy_thresholdingValue_36);
            second_Score_Result = k;
        }
    }

    if (max_Socre_Result!=9 && max_Score>ModelTable_36_Score_Required[max_Socre_Result])
    {
        //"2左环岛", "3右环岛", "4三岔路口", "5十字路口", "12左丁字", "13右丁字", "14T字"
        switch(max_Socre_Result)
        {
            case 0:
                *classification_Result_address = 4;
                break;
            case 1:
                *classification_Result_address = 5;
                break;
            case 2:
                *classification_Result_address = 3;
                break;
            case 3:
                *classification_Result_address = 2;
                break;
            case 4:
                *classification_Result_address = 13;
                break;
            case 5:
                *classification_Result_address = 12;
                break;
            case 6:
                *classification_Result_address = 14;
                break;
            default:
                *classification_Result_address = 9;//未知
        }
    }
    else
    {
        *classification_Result_address = 9;//未知
    }

    if (second_Score_Result!=9 && second_Score>ModelTable_36_Score_Required[second_Score_Result])
    {
        //"2左环岛", "3右环岛", "4三岔路口", "5十字路口", "12左丁字", "13右丁字", "14T字"
        switch(second_Score_Result)
        {
            case 0:
                *classification_Result_2nd_address = 4;
                break;
            case 1:
                *classification_Result_2nd_address = 5;
                break;
            case 2:
                *classification_Result_2nd_address = 3;
                break;
            case 3:
                *classification_Result_2nd_address = 2;
                break;
            case 4:
                *classification_Result_2nd_address = 13;
                break;
            case 5:
                *classification_Result_2nd_address = 12;
                break;
            case 6:
                *classification_Result_2nd_address = 14;
                break;
            default:
                *classification_Result_2nd_address = 9;//未知
        }
    }
    else
    {
        *classification_Result_2nd_address = 9;//未知
    }
}

float Score(uint8 k)
{
    return ((score[k] - ModelTable_36_Score[k]*fuzzy_thresholdingValue_36)/(ModelTable_36_Score[k]*fuzzy_thresholdingValue_36));
}

void Check_Classification(uint8 classification_Result_tmp, uint8 check_counter)
{
    static uint8 Classification_counter=0;
    static uint8 last_classification_Result;

    if(classification_Result_tmp == last_classification_Result)
    {
        Classification_counter++;
        if(Classification_counter>=check_counter){
            classification_Result = classification_Result_tmp;
            Classification_counter =0;
        }
    }
    else
    {
        last_classification_Result = classification_Result_tmp;
        Classification_counter=0;
    }
}

void Set_Search_Range(uint8 row_start,uint8 row_lines,uint8 col_start,uint8 col_lines)
{
    Search_Range[0][0] = row_start;
    Search_Range[0][1] = row_lines;
    Search_Range[1][0] = col_start;
    Search_Range[1][1] = col_lines;
}

void Check(uint8 *classification_Result,uint8 else_result)
{
    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    if (*classification_Result == 13)//右丁字
    {
        if(!(Check_RightP()&&Check_Left_Straight(2,-2,0.4)))
        {
            *classification_Result = else_result;
        }
        else
        {
            flag_For_Right_T = 1;
            flag_For_Left_T = 0;
        }
    }
//    if (*classification_Result == 5)
//    {
//        ;
//    }
    if (*classification_Result == 12)//左丁字
    {
        if(!(Check_LeftP()&&Check_Right_Straight(2,-2,0.4)))
        {
            *classification_Result = else_result;
        }
        else
        {
            flag_For_Right_T = 0;
            flag_For_Left_T = 1;
        }
    }
    if (*classification_Result ==3)//3右环岛
    {
        if((zebra_status != finding && zebra_status != banning)||flag_For_Right_Circle!=0 || !(
                /*(abs(left_width[0]-left_width[1])<=3&&abs(left_width[0]-left_width[2])<=3&&abs(left_width[0]-left_width[3])<=3&&right_width[0]>right_width[1]&&right_width[1]<right_width[2]&&right_width[2]>right_width[3]&&left_width[0]!=-2&&left_width[1]!=-2&&left_width[2]!=-2&&left_width[3]!=-2)*/
              /*||(abs(left_width[0]-left_width[1])<=2&&abs(left_width[0]-left_width[2])<=2&&abs(left_width[0]-left_width[3])<=2&&right_width[0]<right_width[1]&&right_width[1]>right_width[2]&&right_width[2]<right_width[3]&&left_width[0]!=-2&&left_width[1]!=-2&&left_width[2]!=-2&&left_width[3]!=-2)*/Check_RightCircle_New2()&&Check_RightCircle_New4(0.2f)))
        {
            *classification_Result = else_result;
        }
    }
    if (*classification_Result ==2)//2左环岛
    {
        if((zebra_status != finding && zebra_status != banning)||flag_For_Left_Circle!=0 || !(
                /*(abs(right_width[0]-right_width[1])<=3&&abs(right_width[0]-right_width[2])<=3&&abs(right_width[0]-right_width[3])<=3&&left_width[0]>left_width[1]&&left_width[1]<left_width[2]&&left_width[2]>left_width[3]&&left_width[0]!=-2&&left_width[1]!=-2&&left_width[2]!=-2&&left_width[3]!=-2)*/
              /*||(abs(right_width[0]-right_width[1])<=2&&abs(right_width[0]-right_width[2])<=2&&abs(right_width[0]-right_width[3])<=2&&left_width[0]<left_width[1]&&left_width[1]>left_width[2]&&left_width[2]<left_width[3]&&left_width[0]!=-2&&left_width[1]!=-2&&left_width[2]!=-2&&left_width[3]!=-2)*/Check_LeftCircle_New2()&&Check_LeftCircle_New4(0.2f)))
        {
            *classification_Result = else_result;
        }
    }
    if (*classification_Result ==4)//4三岔路口
    {
        if(!(Check_ThreeRoads_New()&&Check_ThreeRoad_New2()))
        {
            *classification_Result = else_result;
        }
    }
    if (*classification_Result ==14)//14 T字
    {
        if((Check_ThreeRoads_New()&&Check_ThreeRoad_New2()))
        {
            *classification_Result = else_result;
        }
//        else if(!(Check_TRoad(0,0.65,3)&&Check_TRoad(1,0.32,3)))
        else if(!(Check_TRoad_New()&&Check_TRoad(0,0.75,3)&&Check_TRoad(1,0.32,3)))
        {
            *classification_Result = else_result;
        }
//        else if(!Check_Up_Straight(2,-2))
//        {
//            *classification_Result = else_result;
//        }
    }
    if (*classification_Result == 9)//9未知
    {



        if (Left_Straight_Score>=3.0f &&Left_Straight_Score>Unknown_Straight_Score+0.7f && Left_Straight_Score>Right_Straight_Score+0.7f)
        {
            *classification_Result = 7;//7靠左
        }
        else if(Right_Straight_Score>=3.0f &&Right_Straight_Score>Unknown_Straight_Score+0.7f && Right_Straight_Score>Left_Straight_Score+0.7f)
        {
            *classification_Result = 8;//8靠右
        }
        else if(Unknown_Straight_Score>=3.0f &&Unknown_Straight_Score>Right_Straight_Score+0.7f && Unknown_Straight_Score>Left_Straight_Score+0.7f)
        {
            *classification_Result = 9;//9未知
        }
        else
        {
            if (Unknown_Straight_Score>=3.0f &&(Unknown_Straight_Score>Right_Straight_Score+0.25f&&Unknown_Straight_Score>Left_Straight_Score+0.25f) &&(Unknown_Straight_Score>Right_Straight_Score+0.6f||Unknown_Straight_Score>Left_Straight_Score+0.6f))
            {
                *classification_Result = 9;
            }
            else
            {
//                if (Left_Straight_Score>=3.0f && Left_Straight_Score>Right_Straight_Score+0.5f)
                if (Left_Straight_Score>=3.0f &&(Left_Straight_Score>Right_Straight_Score+0.25f&&Left_Straight_Score>Unknown_Straight_Score+0.25f) &&(Left_Straight_Score>Right_Straight_Score+0.6f||Left_Straight_Score>Unknown_Straight_Score+0.6f))
                {
                    *classification_Result = 7;//7靠左
                }
//                if (Right_Straight_Score>=3.0f && Right_Straight_Score>Left_Straight_Score+0.5f)
                if (Right_Straight_Score>=3.0f &&(Right_Straight_Score>Left_Straight_Score+0.25f&&Right_Straight_Score>Unknown_Straight_Score+0.25f) &&(Right_Straight_Score>Left_Straight_Score+0.6f||Right_Straight_Score>Unknown_Straight_Score+0.6f))
                {
                    *classification_Result = 8;//8靠右
                }
            }
//            if(Check_Left_Straight_ForRoad(2,-2,0.5) && (Check_Right_Empty(0.5)>Check_Left_Empty(0.5)))
//            {
//                *classification_Result = 7;//7靠左
//            }
//            if(Check_Right_Straight_ForRoad(2,-2,0.5) && (Check_Right_Empty(0.5)<Check_Left_Empty(0.5)))
//            {
//                *classification_Result = 8;//8靠右
//            }
        }

//        if (Check_Left_All_Road(0.9f,6))
//        {
//            *classification_Result = 7;//7靠左
//        }
//
//        if (Check_Right_All_Road(0.9f,6))
//        {
//            *classification_Result = 8;//8靠右
//        }
    }

}








/****************************************************************
 * 函数名: GetBinThreshold_OSTU
 * 功  能: 使用大律法获得二值化阈值
 * 输  入: image    -->    灰度图像
 ****************************************************************/
/* 计算类间方差使用的图像范围, 若使用全图, 依次为0, 0, IMAGE_WIDTH, IMAGE_HEIGHT*/
#define OSTU_START_U (0)
#define OSTU_START_V (20)
#define OSTU_END_U   (X_WIDTH_CAMERA)
#define OSTU_END_V   (Y_WIDTH_CAMERA)
/* 计算类间方差的总像素点数 */
void GetBinThreshold_OSTU(void)
{
    uint16 i, j;
    uint16 histogram[256];
    uint16 backPixel = 0, forePixel;
    uint8 minGray = 0xFF, maxGray = 0, threshold = 0;
    uint32 pixelGraySum = 0, backPixelGraySum = 0, forePixelGraySum = 0;
    float32 diff, interclassVariance, maxInterclassVariance = -1;

    static uint8 flag=0;
    static int OSTU_PIXEL_NUMBER=0;

    /* 初始化灰度直方图 */
    for (i = 0; i < 256; ++i)
    {
        histogram[i] = 0;
    }

    /* 统计灰度级中每个像素在整幅图像中的个数并记录最大和最小灰度 */
    for (i = OSTU_START_V; i < OSTU_END_V; i+=3)
    {
        for (j = OSTU_START_U; j < OSTU_END_U; j+=3)
        {
            ++histogram[mt9v03x_image[i][j]];
            pixelGraySum += mt9v03x_image[i][j];
            if (mt9v03x_image[i][j] < minGray)
            {
                minGray = mt9v03x_image[i][j];
            }
            if (mt9v03x_image[i][j] > maxGray)
            {
                maxGray = mt9v03x_image[i][j];
            }


            if (flag==0)
            {
                OSTU_PIXEL_NUMBER++;
            }
        }
    }

    /* 只有一种或两种灰度 */
    if (minGray == maxGray || minGray+1 == maxGray)
    {
        thresholding_Value =  minGray;
    }


    uint8 min,max;
    /* 求最大类间方差 */
//    if (flag==0)
//    {
        min = minGray;
        max = maxGray;
//    }
//    else
//    {
//        min = minGray>(thresholding_Value-80)?minGray:(thresholding_Value-80);
//        max = maxGray<(thresholding_Value+80)?maxGray:(thresholding_Value+80);
//    }
    for (i = min; i <= max; ++i)
    {
        backPixel += histogram[i];
        forePixel = OSTU_PIXEL_NUMBER - backPixel;
        backPixelGraySum += i * histogram[i];
        forePixelGraySum = pixelGraySum - backPixelGraySum;
        diff = (float)backPixelGraySum/backPixel - (float)forePixelGraySum/forePixel;
        interclassVariance =
               diff * diff  * (float)backPixel * forePixel;// / OSTU_PIXEL_NUMBER / OSTU_PIXEL_NUMBER;
        if (interclassVariance > maxInterclassVariance)
        {
            maxInterclassVariance = interclassVariance;
            threshold = (uint8)i;
        }
    }

    flag=1;
    thresholding_Value = threshold;
}
