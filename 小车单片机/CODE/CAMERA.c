#include "headfile.h"
#include "CAMERA.h"
#include "fastlz.h"//压缩算法
#include <stdlib.h>
#include "OLED.h"


//需要串口通信输出去，但不用传过来的变量
IFX_ALIGN(4) uint8 mt9v03x_image_cutted[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];//裁剪后的原始图像
uint8 classification_Result;

int search_Lines_Straight;//指直线检测的有效扫描行数
float Col_Center[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
int search_Lines;//指Col_Center的有效扫描行数，用于遍历Col_Center


//需要串口通信传过来的变量（必须配以执行变量更新的函数）
uint8 thresholding_Value = 128; //对应的更新函数为：void Set_Thresholding_Value(uint8 val);
float cameraAlphaUpOrDown = 40.0f * 3.1415926 / 2 / 180;//无需校正
float cameraThetaDown = 27.89191f * 3.1415926 / 180;//需要校正
float ratioOfMaxDisToHG = 5.915322f;//仅影响显示距离
float ratioOfPixelToHG = 0.1f;//仅影响分辨率


//不需要传输的其他变量
IFX_ALIGN(4) uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];//原始图像
IFX_ALIGN(4) uint8 mt9v03x_image_cutted_thresholding[Y_WIDTH_CAMERA][X_WIDTH_CAMERA];
IFX_ALIGN(4) uint8 mt9v03x_image_cutted_thresholding_inversePerspective[height_Inverse_Perspective_Max][width_Inverse_Perspective_Max];
int width_Inverse_Perspective;
int height_Inverse_Perspective;

float BayesTable_16[13][CLASS_NUM] = {{-6.212,-9.319,-24.312,-25.426,-3.056,-29.253},
        {25.361,26.676,57.032,54.606,25.934,60.921},
        {30.833,28.652,55.679,57.031,30.260,59.570},
        {-11.965,-9.801,-27.646,-27.080,-5.181,-31.199},
        {-7.657,-8.620,-9.958,-20.068,1.053,-2.612},
        {20.082,7.303,33.639,33.028,19.800,30.603},
        {-7.004,11.375,13.713,21.002,4.998,17.201},
        {-3.465,-3.579,-12.959,-6.872,5.111,0.053},
        {155.160,54.548,219.792,151.069,241.736,245.156},
        {52.537,147.741,143.563,208.364,230.583,233.707},
        {138.356,156.841,144.860,165.326,99.307,119.594},
        {122.261,105.980,128.375,112.617,70.663,89.646},
        {-189.763,-187.540,-328.932,-331.373,-322.336,-380.283}};
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

char *class_Name_Group[CLASS_NUM+5] = {"0左弯", "1右弯", "2左环岛", "3右环岛", "4三岔路口", "5十字路口","6直道","7靠左（临时使用）","8靠右（临时使用）", "9未知","10左直线"};

float arg_Classification_16[16];
float arg_Classification_25[25];


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
    uart_putbuff(DEBUG_UART, *mt9v03x_image_cutted, X_WIDTH_CAMERA*Y_WIDTH_CAMERA);  //发送压缩后的图像
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
    uint32 len = X_WIDTH_CAMERA*Y_WIDTH_CAMERA/8; //要求X_WIDTH_CAMERA*Y_WIDTH_CAMERA刚好是8的倍数
    uint8 *buff = mt9v03x_image_cutted_thresholding;
    while(len)
    {

        uint8 dat = (*buff<<7)
                | (*(buff+1)<<6)
                | (*(buff+2)<<5)
                | (*(buff+3)<<4)
                | (*(buff+4)<<3)
                | (*(buff+5)<<2)
                | (*(buff+6)<<1)
                | (*(buff+7));
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

void UART_ColCenter(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x09);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Center[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)round(Col_Center[i]));
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x09);
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
void Get_Cutted_Image(void)
{
    int origin_i=0,origin_j=0;
    origin_i = (MT9V03X_H - Y_WIDTH_CAMERA)/2;
    origin_j = (MT9V03X_W - X_WIDTH_CAMERA)/2;
    for(int i = 0; i < Y_WIDTH_CAMERA; i++)
    {
        origin_j = (MT9V03X_W - X_WIDTH_CAMERA)/2;
        for(int j = 0; j < X_WIDTH_CAMERA; j++)
        {
            mt9v03x_image_cutted[i][j] = mt9v03x_image[origin_i][origin_j];
            origin_j++;
        }
        origin_i+=2;
    }
}



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

void Get_Thresholding_Image(void)
{
    //Kmeans法更新二值化阈值
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
                float min_distance = fabs(mt9v03x_image_cutted[i][j]-m[0][0]);
                int min_distance_class = 0;
                for (int k=1;k<KMEANS_K;k++)
                {
                    if (fabs(mt9v03x_image_cutted[i][j]-m[k][0]) < min_distance)
                    {
                        min_distance = fabs(mt9v03x_image_cutted[i][j]-m[k][0]);
                        min_distance_class = k;
                    }
                }
                m[min_distance_class][1] = m[min_distance_class][1]+1;
                m[min_distance_class][2] = (m[min_distance_class][1]-1)/m[min_distance_class][1]*m[min_distance_class][2] + mt9v03x_image_cutted[i][j]/m[min_distance_class][1];
            }
        }
        for (int i = 0;i<KMEANS_K;i++)
        {
            m[i][0] = m[i][2];
        }
    }
    thresholding_Value = (uint8)(0.5*(m[0][0]+m[1][0]));

    for(int j = 0; j < Y_WIDTH_CAMERA; j++)
    {
        for(int i = 0; i < X_WIDTH_CAMERA; i++)
        {
            mt9v03x_image_cutted_thresholding[j][i] = mt9v03x_image_cutted[j][i]>thresholding_Value;
        }
    }
}

void Get_Inverse_Perspective_Image(void)
{
    int ratio=2;
    width_Inverse_Perspective = (int)round(2 * (X_WIDTH_CAMERA*1.0) / (ratio*Y_WIDTH_CAMERA*1.0) * tan(cameraAlphaUpOrDown) / cos(cameraThetaDown) * ratioOfMaxDisToHG / ratioOfPixelToHG);
    height_Inverse_Perspective = (int)round(ratioOfMaxDisToHG / ratioOfPixelToHG);

    for (int j_Processed = 0;j_Processed<height_Inverse_Perspective;j_Processed++)
    {
        float y_Processed = (float)(height_Inverse_Perspective - 1 - j_Processed);
        float y = (tan(cameraThetaDown)-1.0/y_Processed/ratioOfPixelToHG)*ratio*(Y_WIDTH_CAMERA*1.0)/2.0/tan(cameraAlphaUpOrDown);
        int j = (int)round(-y + (ratio*Y_WIDTH_CAMERA*1.0-1)/2.0);
        if (j>=0 && j<ratio*Y_WIDTH_CAMERA*1.0)
        {
            for (int i_Processed = 0;i_Processed<width_Inverse_Perspective;i_Processed++)
            {
                float x_Processed = (float)i_Processed - ((float)width_Inverse_Perspective-1)/2.0;
                float x = x_Processed*(ratio*Y_WIDTH_CAMERA*1.0/2.0/tan(cameraAlphaUpOrDown)*sin(cameraThetaDown)-y*cos(cameraThetaDown))*ratioOfPixelToHG;
                int i = (int)round(x + (X_WIDTH_CAMERA*1.0-1)/2.0);
                if (i>=0 && i<X_WIDTH_CAMERA*1.0)
                {
                    mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = mt9v03x_image_cutted_thresholding[j/ratio][i];
                }
                else
                {
                    mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = 255;
                }
            }
        }
        else
        {
            for (int i_Processed = 1;i_Processed<width_Inverse_Perspective;i_Processed++)
            {
                mt9v03x_image_cutted_thresholding_inversePerspective[j_Processed][i_Processed] = 255;
            }
        }
    }
}

void Get16(float* arg)
{
    int col_edge[5],row_edge[5];
    col_edge[0] = 0;
    col_edge[1] = width_Inverse_Perspective/4;
    col_edge[2] = 2*col_edge[1];
    col_edge[3] = 3*col_edge[1];
    col_edge[4] = width_Inverse_Perspective-1;
    row_edge[0] = 0;
    row_edge[1] = height_Inverse_Perspective/4;
    row_edge[2] = 2*row_edge[1];
    row_edge[3] = 3*row_edge[1];
    row_edge[4] = height_Inverse_Perspective-1;

    int white_cnt[16] = {0};
    int black_cnt[16] = {0};
    for (int i = 0;i<height_Inverse_Perspective;i++)
    {
        for (int j = 0;j<width_Inverse_Perspective;j++)
        {
            white_cnt[4*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[4*(i/row_edge[1]) + j/col_edge[1]];
            black_cnt[4*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[4*(i/row_edge[1]) + j/col_edge[1]];
        }
    }
    for (int i = 0;i<16;i++)
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

uint8 Classification_16(void)
{
    //float arg_Classification_16[16];
    Get16(arg_Classification_16);
    float classification_Data[CLASS_NUM] = {0};
    float classification_Data_max1=0;
    uint8 classification_Result_temp1=0;
    float classification_Data_max2=0;
    uint8 classification_Result_temp2=0;
    for (uint8 i = 0;i<CLASS_NUM;i++)
    {
        classification_Data[i] = arg_Classification_16[0]*BayesTable_16[0][i]+
                                arg_Classification_16[1]*BayesTable_16[1][i]+
                                arg_Classification_16[2]*BayesTable_16[2][i]+
                                arg_Classification_16[3]*BayesTable_16[3][i]+
                                arg_Classification_16[4]*BayesTable_16[4][i]+
                                arg_Classification_16[5]*BayesTable_16[5][i]+
                                arg_Classification_16[6]*BayesTable_16[6][i]+
                                arg_Classification_16[7]*BayesTable_16[7][i]+
                                arg_Classification_16[9]*BayesTable_16[8][i]+
                                arg_Classification_16[10]*BayesTable_16[9][i]+
                                arg_Classification_16[13]*BayesTable_16[10][i]+
                                arg_Classification_16[14]*BayesTable_16[11][i]+
                                BayesTable_16[12][i];

        if (classification_Data[i]>classification_Data_max1)
        {
            classification_Data_max2 = classification_Data_max1;
            classification_Result_temp2 = classification_Result_temp1;
            classification_Data_max1 = classification_Data[i];
            classification_Result_temp1 = i;
        }
        else if (classification_Data[i]>classification_Data_max2)
        {
            classification_Data_max2 = classification_Data[i];
            classification_Result_temp2 = i;
        }
    }
    if ((classification_Data_max1-classification_Data_max2) > CLASSIFICATION_16_VALID)
    {
        return classification_Result_temp1;
    }
    else
    {
        return 9;//9代表未知
    }
}

void Get25(float* arg)
{
    int col_edge[6],row_edge[6];
    col_edge[0] = 0;
    col_edge[1] = width_Inverse_Perspective/5;
    col_edge[2] = 2*col_edge[1];
    col_edge[3] = 3*col_edge[1];
    col_edge[4] = 4*col_edge[1];
    col_edge[5] = width_Inverse_Perspective-1;
    row_edge[0] = 0;
    row_edge[1] = height_Inverse_Perspective/5;
    row_edge[2] = 2*row_edge[1];
    row_edge[3] = 3*row_edge[1];
    row_edge[4] = 4*row_edge[1];
    row_edge[5] = height_Inverse_Perspective-1;

    int white_cnt[25] = {0};
    int black_cnt[25] = {0};
    for (int i = 0;i<height_Inverse_Perspective;i++)
    {
        for (int j = 0;j<width_Inverse_Perspective;j++)
        {
            white_cnt[5*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1) + white_cnt[5*(i/row_edge[1]) + j/col_edge[1]];
            black_cnt[5*(i/row_edge[1]) + j/col_edge[1]] = (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0) + black_cnt[5*(i/row_edge[1]) + j/col_edge[1]];
        }
    }
    for (int i = 0;i<25;i++)
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

uint8 Classification_25(void)
{
    Get25(arg_Classification_25);
    float classification_Data[CLASS_NUM] = {0};
    float classification_Data_max1=0;
    uint8 classification_Result_temp1=0;
    float classification_Data_max2=0;
    uint8 classification_Result_temp2=0;
    for (uint8 i = 0;i<CLASS_NUM;i++)
    {
        classification_Data[i] = arg_Classification_25[0]*BayesTable_25[0][i]+
                arg_Classification_25[1]*BayesTable_25[1][i]+
                arg_Classification_25[2]*BayesTable_25[2][i]+
                arg_Classification_25[3]*BayesTable_25[3][i]+
                arg_Classification_25[4]*BayesTable_25[4][i]+
                arg_Classification_25[5]*BayesTable_25[5][i]+
                arg_Classification_25[6]*BayesTable_25[6][i]+
                arg_Classification_25[7]*BayesTable_25[7][i]+
                arg_Classification_25[8]*BayesTable_25[8][i]+
                arg_Classification_25[9]*BayesTable_25[9][i]+
                arg_Classification_25[11]*BayesTable_25[10][i]+
                arg_Classification_25[12]*BayesTable_25[11][i]+
                arg_Classification_25[13]*BayesTable_25[12][i]+
                arg_Classification_25[16]*BayesTable_25[13][i]+
                arg_Classification_25[17]*BayesTable_25[14][i]+
                arg_Classification_25[18]*BayesTable_25[15][i]+
                BayesTable_25[16][i];


        if (classification_Data[i]>classification_Data_max1)
        {
            classification_Data_max2 = classification_Data_max1;
            classification_Result_temp2 = classification_Result_temp1;
            classification_Data_max1 = classification_Data[i];
            classification_Result_temp1 = i;
        }
        else if (classification_Data[i]>classification_Data_max2)
        {
            classification_Data_max2 = classification_Data[i];
            classification_Result_temp2 = i;
        }
    }
    if ((classification_Data_max1-classification_Data_max2) > CLASSIFICATION_25_VALID)
    {
        return classification_Result_temp1;
    }
    else
    {
        return 9;//9代表未知
    }
}


int Check_Straight(void)
{
    int start_Row = height_Inverse_Perspective-1;
    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};
    int Col_Left[height_Inverse_Perspective_Max] = {-2};
    int Col_Right[height_Inverse_Perspective_Max] = {-2};
    search_Lines_Straight=(int)(3/ratioOfPixelToHG); //表示直线扫描范围为车前3.0倍摄像头高度
    for (int i=0;i<search_Lines_Straight;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines_Straight;i++)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)
        {
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
            {
                start_Col[0] = start_Col[0] - 1;
            }
            start_Col[0] = start_Col[0] + 1;
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)
            {
                Col_Left[i] = start_Col[0];
            }
        }
        else
        {
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)
            {
                Col_Left[i] = start_Col[0];
            }
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)
        {
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];
            }
        }
        else
        {
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1 && start_Col[1]<=width_Inverse_Perspective-2)
            {
                start_Col[1] = start_Col[1] + 1;
            }
            start_Col[1] = start_Col[1] - 1;
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];
            }
        }
        start_Row = start_Row - 1;
    }

    int max_Col_Left = 0;
    int min_Col_Left = width_Inverse_Perspective;
    int detect_Left = 0;
    int max_Col_Right = 0;
    int min_Col_Right = width_Inverse_Perspective;
    int detect_Right = 0;
    for (int i=0;i<search_Lines_Straight;i++)
    {
        if (Col_Left[i] != -2)
        {
            detect_Left = 1;
            if (Col_Left[i] > max_Col_Left)
            {
                max_Col_Left = Col_Left[i];
            }
            if (Col_Left[i] < min_Col_Left)
            {
                min_Col_Left = Col_Left[i];
            }
        }
        else if (detect_Left == 1)
        {
            return 0;
        }

        if (Col_Right[i] != -2)
        {
            detect_Right = 1;
            if (Col_Right[i] > max_Col_Right)
            {
                max_Col_Right = Col_Right[i];
            }
            if (Col_Right[i] < min_Col_Right)
            {
                min_Col_Right = Col_Right[i];
            }
        }
        else if (detect_Right == 1)
        {
            return 0;
        }
    }
    return ((max_Col_Left - min_Col_Left <= STRAIGHT_CONDITION && max_Col_Left >= min_Col_Left)
            && (max_Col_Right - min_Col_Right <= STRAIGHT_CONDITION && max_Col_Right >= min_Col_Right)
            && max_Col_Left < min_Col_Right);
}


int Check_Left_Straight(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
    int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//在当前的行里，如果左线或右线有一个是255区域（未知区域），说明还没有进入到真正的视角区域（0区域或1区域）；或者如果左线跑到右线的右边去了，则说明不是道路了
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //如果左线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
            {
                start_Col[0] = start_Col[0] - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }
            start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
            {
                Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
            }
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
            {
                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
            }
        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备
    }

    start_Row = height_Inverse_Perspective-1;
    int d_Col_Left;
    int d_Col_Left_Num=0;

    int i;
    for (i=0;i<search_Lines;i++)
    {
        if (Col_Left[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
            {
                d_Col_Left_Num = search_Lines-i-1;
                break;
            }
            else
            {
                return 0;
            }
        }
        start_Row = start_Row - 1;
    }
    for (int j=0;j<d_Col_Left_Num;j++)
    {
        d_Col_Left = Col_Left[i+1]-Col_Left[i];
        i++;
        if (d_Col_Left < 0 || d_Col_Left > 1)
        {
            return 0;
        }
    }
    return 1;
}


void DrawCenterLine(void)
{
    search_Lines = ((int)(height_Inverse_Perspective/(10.0f*ratioOfPixelToHG)));//一共要扫描多少行，最大是图片宽
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = -2;
    }

    // 对于6直道，可以采用，特征是滤波是较大正数，用于避免中心线有过大的波动
    if (classification_Result == 6)
    {
        DrawCenterLinewithConfig(0.7);
    }
    // 对于5十字路口，可以采用，特征是使用卷积核判断两个拐点
    else if (classification_Result == 5)
    {
        DrawCenterLinewithConfig_CrossRoad();
    }
    // 对于4三岔路口、8靠右（临时使用）可以采用，特征是靠右行驶
    else if (classification_Result == 4 || classification_Result == 8)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于7靠左（临时使用）可以采用，特征是靠左行驶
    else if (classification_Result == 7)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    // 对于0左弯、1右弯以及剩余还没写好的道路元素，可以采用，特征是滤波是负数，用于超前转向，以免冲出弯道
    else
    {
        DrawCenterLinewithConfig(0);
    }
}

//filter滤波系数，正数时是低通滤波，负数时相当于高通滤波
void DrawCenterLinewithConfig(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
    int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//在当前的行里，如果左线或右线有一个是255区域（未知区域），说明还没有进入到真正的视角区域（0区域或1区域）；或者如果左线跑到右线的右边去了，则说明不是道路了
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //如果左线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
            {
                start_Col[0] = start_Col[0] - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }
            start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
            {
                Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
            }
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
            {
                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
            }
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//如果右线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
            }
        }
        else//如果右线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1 && start_Col[1]<=width_Inverse_Perspective-2)
            {
                start_Col[1] = start_Col[1] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }
            start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//只有是0区域的才可以将列号存储到右线里
            }
        }
        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备

        //下面是中心线计算
        if (start_Col[0]>start_Col[1])
        {
            continue;
        }
        //中心线计算有4种情况：左线合法(!=-2)或非法(==-2)）、右线合法(!=-2)或非法(==-2)）
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //左线合法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一个中心线也合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //根据上一次的中心线、这一次左右线中值，用滤波计算这次的中心线
            }
            else //如果上一个中心线不合法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //假定上一次中心线在正中间，根据上一次中心线、这一次左右线中值，用滤波计算这次的中心线
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //左线非法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一次中心线合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //根据上一次的中心线、假定的这一次中心线（保持上一次和右线的距离），用滤波计算这次的中心线
            }
            else //如果上一次中心线非法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //这一次中心线直接假定为正中间
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //左线合法，右线非法，与上文类似
        {
            if (Col_Center[i-1]!=-2)
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
            else
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
        }
        else //左线非法，右线非法
        {

        }
        //中心线计算完毕
    }
}

void DrawCenterLinewithConfig_RightBased(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
    int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//在当前的行里，如果左线或右线有一个是255区域（未知区域），说明还没有进入到真正的视角区域（0区域或1区域）；或者如果左线跑到右线的右边去了，则说明不是道路了
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//如果右线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
            if (cnt_temp>road_width)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
            }
        }
        else//如果右线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1 && start_Col[1]<=width_Inverse_Perspective-2)
            {
                start_Col[1] = start_Col[1] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>road_width)
            {
                break;
            }
            start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//只有是0区域的才可以将列号存储到右线里
            }
        }
        start_Col[0] = start_Col[1]-road_width;//左线的初始扫描点是右线往左road_width
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //如果左线发现1区域（道路）
        {
            Col_Left[i] = start_Col[0];//第一次之后，剩余时候发现1区域就保持右线往左road_width
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
            if (cnt_temp>road_width)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
            {
                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
            }
        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备

        //下面是中心线计算
        if (start_Col[0]>start_Col[1])
        {
            continue;
        }
        //中心线计算有4种情况：左线合法(!=-2)或非法(==-2)）、右线合法(!=-2)或非法(==-2)）
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //左线合法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一个中心线也合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //根据上一次的中心线、这一次左右线中值，用滤波计算这次的中心线
            }
            else //如果上一个中心线不合法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //假定上一次中心线在正中间，根据上一次中心线、这一次左右线中值，用滤波计算这次的中心线
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //左线非法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一次中心线合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //根据上一次的中心线、假定的这一次中心线（保持上一次和右线的距离），用滤波计算这次的中心线
            }
            else //如果上一次中心线非法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //这一次中心线直接假定为正中间
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //左线合法，右线非法，与上文类似
        {
            if (Col_Center[i-1]!=-2)
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
            else
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
        }
        else //左线非法，右线非法
        {

        }
        //中心线计算完毕
    }
}

void DrawCenterLinewithConfig_LeftBased(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
    int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//在当前的行里，如果左线或右线有一个是255区域（未知区域），说明还没有进入到真正的视角区域（0区域或1区域）；或者如果左线跑到右线的右边去了，则说明不是道路了
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //如果左线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
            {
                start_Col[0] = start_Col[0] - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>20)
            {
                break;
            }
            start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
            {
                Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
            }
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
            if (cnt_temp>20)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
            {
                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
            }
        }
        start_Col[1] = start_Col[0]+road_width;//右线的初始扫描点是左线往右road_width
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//如果右线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
            if (cnt_temp>20)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
            }
        }
        else//如果右线发现1区域（道路）
        {
            Col_Right[i] = start_Col[1];//第一次之后，剩余时候发现1区域就保持左线往右road_width
        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备

        //下面是中心线计算
        if (start_Col[0]>start_Col[1])
        {
            continue;
        }
        //中心线计算有4种情况：左线合法(!=-2)或非法(==-2)）、右线合法(!=-2)或非法(==-2)）
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //左线合法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一个中心线也合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //根据上一次的中心线、这一次左右线中值，用滤波计算这次的中心线
            }
            else //如果上一个中心线不合法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //假定上一次中心线在正中间，根据上一次中心线、这一次左右线中值，用滤波计算这次的中心线
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //左线非法，右线合法
        {
            if (Col_Center[i-1]!=-2) //如果上一次中心线合法
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //根据上一次的中心线、假定的这一次中心线（保持上一次和右线的距离），用滤波计算这次的中心线
            }
            else //如果上一次中心线非法
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //这一次中心线直接假定为正中间
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //左线合法，右线非法，与上文类似
        {
            if (Col_Center[i-1]!=-2)
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
            else
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Left[i]+road_width/2.0f);
            }
        }
        else //左线非法，右线非法
        {

        }
        //中心线计算完毕
    }
}

void DrawCenterLinewithConfig_CrossRoad(void)
{
    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
    int Conv_Core[2][3][3] = {{{1,1,-1},{1,1,-1},{-1,-1,-1}},{{-1,1,1},{-1,1,1},{-1,-1,-1}}};
    //Conv_Core(1).core = [1 1 -1
//                         1 1 -1
//                        -1 -1 -1];
    //Conv_Core(2).core = [-1 1 1
//                         -1 1 1
//                        -1 -1 -1];

    int Conv_Score_max[2] = {-9,-9};
    int Conv_Score_max_i[2] = {0,0};
    int Conv_Score_max_j[2] = {0,0};
    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
    {
        for (int j=1;j<width_Inverse_Perspective-1;j++)
        {
            // 对于每个中心点(i,j)，计算2类卷积值，分别取最大值保留
            int Conv_Score[2] = {0,0};//存储2类卷积结果
            int flag = 1;//卷积合不合法
            for (int k=0;k<2;k++)
            {
                for (int ii=0;ii<3;ii++)
                {
                    for (int jj=0;jj<3;jj++)
                    {
                        if (mt9v03x_image_cutted_thresholding_inversePerspective[i-1+ii][j-1+jj] == 255)
                        {
                            flag = 0;
                            break;
                        }
                        if (mt9v03x_image_cutted_thresholding_inversePerspective[i-1+ii][j-1+jj] == 0)
                        {
                            Conv_Score[k] = Conv_Score[k] +1*Conv_Core[k][ii][jj];
                        }
                        else
                        {
                            Conv_Score[k] = Conv_Score[k] +(-1)*Conv_Core[k][ii][jj];
                        }
                    }
                    if (flag == 0)
                    {
                        break;
                    }
                }
                if (flag == 0)
                {
                    break;
                }
            }
            if (flag == 0)
            {
                continue;
            }
            if (Conv_Score[0] > Conv_Score_max[0] && j<width_Inverse_Perspective*0.6)
            {
                Conv_Score_max[0] = Conv_Score[0];
                Conv_Score_max_i[0] = i;
                Conv_Score_max_j[0] = j;
            }
            if (Conv_Score[1] > Conv_Score_max[1] && j>width_Inverse_Perspective*0.4)
            {
                Conv_Score_max[1] = Conv_Score[1];
                Conv_Score_max_i[1] = i;
                Conv_Score_max_j[1] = j;
            }
        }
    }

    //存储底部中点坐标
    int i;
    for (i=0;i<search_Lines;i++) //寻找视野底部
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[search_Lines-1-i][width_Inverse_Perspective/2] != 255)
        {
            Col_Center[i] = (float)(width_Inverse_Perspective/2);
            break;
        }
    }


    int firsti = i;

    //补全拐点间中线和底部中线之间的连线
    float k = ((Conv_Score_max_j[0] + Conv_Score_max_j[1])/2.0f  -  width_Inverse_Perspective/2)/(search_Lines-firsti-1 - (Conv_Score_max_i[0] + Conv_Score_max_i[1])/2.0f );

    for (int i=1;i<search_Lines - firsti;i++)
    {
        Col_Center[i+firsti] = width_Inverse_Perspective/2 + k*i;
    }
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
