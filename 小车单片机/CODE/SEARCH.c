#include "headfile.h"
#include "CAMERA.h"
#include "SEARCH.h"
#include "fastlz.h"//压缩算法
#include <stdlib.h>
#include "OLED.h"
#include "ICM.h"


//需要串口通信输出去，但不用传过来的变量
float Col_Center[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
float Col_Center_Backup[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2

float left_empty,right_empty;

//需要串口通信传过来的变量（必须配以执行变量更新的函数）


//不需要传输的其他变量
int road_width; //道路实际宽度0.4m

int search_Lines_Straight;//指直线检测的有效扫描行数
int search_Lines;//指Col_Center的有效扫描行数，用于遍历Col_Center

float threeRoads_RightTime = 0.15f;
float rightCircle_RightTime = 0.1f;
float rightCircle_LeftTime = 0.2f;
float rightCircle_BannedTime = 3.0f;
float T_Time = 1.0f;

int8 Circle_target_down[2] = {1,8};
int8 Circle_target_up[2] = {5,18};
int8 Circle_d_target[2] = {0,17};//{3,15};
//int8 Circle_target_down[2] = {2,8};
//int8 Circle_target_up[2] = {5,15};
//int8 Circle_d_target[2] = {3,12};
int8 Circle_lines = 3;

//int8 ThreeRoads_target_down[2] = {2,10};//{1,10};
//int8 ThreeRoads_target_up[2] = {5,15};//{5,16};//{5,19};
//int8 ThreeRoads_d_target[2] = {0,13};//{1,14};//{1,9};
int8 ThreeRoads_lines[2] = {5,9};
//11、20是假三岔

int8 last_angle_down;
int8 last_angle_up;
int8 last_angle_upup;

int8 first_Dot[2];
int8 second_Dot[2];
int8 third_Dot[2];
float arccosValue;

uint8 DrawLineFilter = 0;

float Left_Straight_Score=0,Unknown_Straight_Score=0,Right_Straight_Score=0;


uint8 flag_For_T = 0;
uint8 flag_For_ThreeRoad = 0;

uint8 White2Black_cnt = 0;
enum ZEBRA_STATUS zebra_status = starting;
uint8 Zebra_times=0;
uint8 Zebra_times_Max=2;
int zebra_direction = 0;
int zebra_start_direction = 1;

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

void UART_ColLeft(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x10);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Left[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)Col_Left[i]);
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x10);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void UART_ColRight(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x11);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Right[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)Col_Right[i]);
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x11);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void DrawCenterLine(void)
{
    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    search_Lines = height_Inverse_Perspective;//一共要扫描多少行，最大是图片宽
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = -2;
    }

    if(is_Slope==1)
    {
        DrawCenterLinewithConfig(0.7);
    }
    // 对于6直道，可以采用，特征是滤波是较大正数，用于避免中心线有过大的波动
    else if (classification_Result == 6)
    {
        DrawCenterLinewithConfig(0.7);
    }
    // 对于5十字路口，可以采用，特征是使用卷积核判断两个拐点
    else if (classification_Result == 5)
    {
        DrawCenterLinewithConfig_CrossRoad();
    }
    // 对于4三岔路口可以采用，特征是靠右行驶
    else if (classification_Result == 4)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于8靠右（临时使用）可以采用，特征是靠右行驶
    else if (classification_Result == 8)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于3右环岛可以采用，特征是靠右行驶
    else if (classification_Result == 3)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于11右直线可以采用，特征是靠右行驶
    else if (classification_Result == 11)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于2左环岛可以采用，特征是靠左行驶
    else if (classification_Result == 2)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    // 对于10左直线可以采用，特征是靠左行驶
    else if (classification_Result == 10)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    // 对于7靠左（临时使用）可以采用，特征是靠左行驶
    else if (classification_Result == 7)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    else if (classification_Result == 13)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    else if (classification_Result == 12)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于0左弯、1右弯以及剩余还没写好的道路元素，可以采用，特征是滤波是负数，用于超前转向，以免冲出弯道
    else
    {
        DrawCenterLinewithConfig(0);
    }
}

uint8 Check_Straight(float ratio)
{
    int start_Row = height_Inverse_Perspective-1;
    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};

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
    for (int i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
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


uint8 Check_Left_Straight_ForRoad(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
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
            if (cnt_temp>2*road_width)
            {
                break;
            }
            start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
//            {
                Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
//            }
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>2*road_width)
            {
                break;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
//            {
                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
//            }
        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Left;
    int d_Col_Left_Num=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Left[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
            {
                d_Col_Left_Num = height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Left_Num<=0.8f*ratio*Search_Range[ROW][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Col_Left_Num;j++)
    {
        d_Col_Left = Col_Left[i+1]-Col_Left[i];
        i++;
        if (d_Col_Left < min_d_Col_Left || d_Col_Left > max_d_Col_Left)
        {
            return 0;
        }
    }
    return 1;
}


uint8 Check_Right_Straight_ForRoad(int8 max_d_Col_Right, int8 min_d_Col_Right, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
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
            }
            if (cnt_temp>2*road_width)
            {
                break;
            }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
//            {
                Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
//            }
        }
        else//如果右线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1 && start_Col[1]<=width_Inverse_Perspective-2)
            {
                start_Col[1] = start_Col[1] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>2*road_width)
            {
                break;
            }
            start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
//            {
                Col_Right[i] = start_Col[1];//只有是0区域的才可以将列号存储到右线里
//            }
        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Right;
    int d_Col_Right_Num=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=0.8f*ratio*Search_Range[ROW][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Col_Right_Num;j++)
    {
        d_Col_Right = Col_Right[i+1]-Col_Right[i];
        i++;
        if (d_Col_Right < min_d_Col_Right || d_Col_Right > max_d_Col_Right)
        {
            return 0;
        }
    }
    return 1;
}

float Check_Left_Empty(float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
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
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int cnt=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1.0f-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Left[i] ==-2)
        {
           cnt++;
        }
    }
    left_empty = cnt/(Search_Range[ROW][LINES]*ratio);
    return cnt/(Search_Range[ROW][LINES]*ratio);
}

float Check_Right_Empty(float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
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
            }
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int cnt=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1.0f-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Right[i] ==-2)
        {
           cnt++;
        }
    }
    right_empty = cnt/(Search_Range[ROW][LINES]*ratio);
    return cnt/(Search_Range[ROW][LINES]*ratio);
}


//filter滤波系数，正数时是低通滤波，负数时相当于高通滤波
void DrawCenterLinewithConfig(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始

    int flag_left=0;
    int flag_right=0;

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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>2*road_width)
//                {
//                    break;
//                }
                start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域

                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                    if (start_Col_temp>start_Col[0])
                    {
                        start_Col[0] = start_Col_temp;
                        Col_Left[i] = start_Col[0];
                        flag_left = 1;
                    }
                    else
                    {
                        flag_left = 0;
                        start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                        flag_left = 1;
                        Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                    }
                }
            }
            else
            {
                if (cnt_temp>2*road_width)
                {
                    break;
                }
                start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域

                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
                {
                   Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                }
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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>2*road_width)
//                {
//                    break;
//                }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                    if (start_Col_temp>start_Col[0])
                    {
                        start_Col[0] = start_Col_temp;
                        Col_Left[i] = start_Col[0];
                        flag_left = 1;
                    }
                    else
                    {
                        flag_left = 0;
                        start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                        flag_left = 1;
                        Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                    }
                }

            }
            else
            {
                if (cnt_temp>2*road_width)
                {
                    break;
                }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
                {
                    Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
                }
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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>2*road_width)
//                {
//                    break;
//                }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                    if (start_Col_temp<start_Col[1])
                    {
                        start_Col[1] = start_Col_temp;
                        Col_Right[i] = start_Col[1];
                        flag_right = 1;
                    }
                    else
                    {
                        flag_right = 0;
                        start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                        flag_right = 1;
                        Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                    }
                }
            }
            else
            {
                if (cnt_temp>2*road_width)
                {
                    break;
                }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
                {
                    Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                }
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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>2*road_width)
//                {
//                    break;
//                }
                start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                    if (start_Col_temp<start_Col[1])
                    {
                        start_Col[1] = start_Col_temp;
                        Col_Right[i] = start_Col[1];
                        flag_right = 1;
                    }
                    else
                    {
                        flag_right = 0;
                        start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                        flag_right = 1;
                        Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                    }
                }

            }
            else
            {
                if (cnt_temp>2*road_width)
                {
                    break;
                }
                start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
                {
                    Col_Right[i] = start_Col[1];//只有是0区域的才可以将列号存储到右线里
                }
            }


        }
        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备

        //下面是中心线计算
//        if (start_Col[0]>start_Col[1] || start_Col[1] - start_Col[0] > 2*road_width)
        if (start_Col[0]>start_Col[1])
        {
            continue;
        }
        //中心线计算有4种情况：左线合法(!=-2)或非法(==-2)）、右线合法(!=-2)或非法(==-2)）
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //左线合法，右线合法
        {
            road_width = Col_Right[i] - Col_Left[i];
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
            if (Col_Center[i-1]!=-2)
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(width_Inverse_Perspective/2);
            }
            else
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(width_Inverse_Perspective/2);
            }
        }
        //中心线计算完毕
    }
}

void DrawCenterLinewithConfig_RightBased(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始

    int flag_right=0;


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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
//                {
//                    break;
//                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                    if (start_Col_temp<start_Col[1])
                    {
                        start_Col[1] = start_Col_temp;
                        Col_Right[i] = start_Col[1];
                        flag_right = 1;
                    }
                    else
                    {
                        flag_right = 0;
                        start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                        flag_right = 1;
                        Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                    }
                }

            }
            else
            {
                if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
                {
                    break;
                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
                {
                    Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                }
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

            if (DrawLineFilter==1)
            {
//                if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
//                {
//                    break;
//                }
                start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                    if (start_Col_temp<start_Col[1])
                    {
                        start_Col[1] = start_Col_temp;
                        Col_Right[i] = start_Col[1];
                        flag_right = 1;
                    }
                    else
                    {
                        flag_right = 0;
                        start_Col_temp = Filter_Col_Right(flag_right,start_Col[1],0.01f);
                        flag_right = 1;
                        Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
                    }
                }

            }
            else
            {
                if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
                {
                    break;
                }
                start_Col[1] = start_Col[1] - 1;//则右线持续向右扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
                {
                    Col_Right[i] = start_Col[1];//只有是0区域的才可以将列号存储到右线里
                }
            }


        }
        start_Col[0] = start_Col[1]-road_width;//左线的初始扫描点是右线往左road_width

        Col_Left[i] = start_Col[0];

//        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //如果左线发现1区域（道路）
//        {
//            Col_Left[i] = start_Col[0];//第一次之后，剩余时候发现1区域就保持右线往左road_width
//        }
//        else if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0)//如果左线发现0区域（背景）
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 1 && start_Col[0]<=width_Inverse_Perspective-2)
//            {
//                start_Col[0] = start_Col[0] + 1;
//                cnt_temp = cnt_temp + 1;
//            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
//            if (cnt_temp>2*road_width)
//            {
//                break;
//            }
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
//            {
//                Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
//            }
//        }
//        else
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 1 && start_Col[0]<=width_Inverse_Perspective-2)
//            {
//                start_Col[0] = start_Col[0] + 1;
//                cnt_temp = cnt_temp + 1;
//            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
//            if (cnt_temp>2*road_width)
//            {
//                break;
//            }
//        }

        start_Row = start_Row - 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备

        //下面是中心线计算
        if (start_Col[0]>start_Col[1] || start_Col[1] - start_Col[0] > 2*road_width)
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

    int flag_left=0;

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

            if (DrawLineFilter==1)
            {

//                if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
//                {
//                    break;
//                }
                start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                    if (start_Col_temp>start_Col[0])
                    {
                        start_Col[0] = start_Col_temp;
                        Col_Left[i] = start_Col[0];
                        flag_left = 1;
                    }
                    else
                    {
                        flag_left = 0;
                        start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                        flag_left = 1;
                        Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                    }
                }

            }
            else
            {
                if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
                {
                    break;
                }
                start_Col[0] = start_Col[0] + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//查看此时是否是0区域（背景）
                {
                    Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                }
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

            if (DrawLineFilter==1)
            {

//                if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
//                {
//                    break;
//                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
                {
                    int start_Col_temp;
                    start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                    if (start_Col_temp>start_Col[0])
                    {
                        start_Col[0] = start_Col_temp;
                        Col_Left[i] = start_Col[0];
                        flag_left = 1;
                    }
                    else
                    {
                        flag_left = 0;
                        start_Col_temp = Filter_Col_Left(flag_left,start_Col[0],0.01f);
                        flag_left = 1;
                        Col_Left[i] = start_Col[0];//只有是0区域的才可以将列号存储到左线里
                    }
                }

            }
            else
            {
                if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
                {
                    break;
                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//查看此时是否是1区域（道路）
                {
                    Col_Left[i] = start_Col[0];//只有是1区域的才可以将列号存储到左线里
                }
            }


        }
        start_Col[1] = start_Col[0]+road_width;//右线的初始扫描点是左线往右road_width


        Col_Right[i] = start_Col[1];

//        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//如果右线发现0区域（背景）
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 1 && start_Col[1]>=1)
//            {
//                start_Col[1] = start_Col[1] - 1;
//                cnt_temp = cnt_temp + 1;
//            }//则右线持续向左扫描直到不再是0区域（背景），有可能是1或255区域
//            if (cnt_temp>2*road_width)
//            {
//                break;
//            }
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
//            {
//                Col_Right[i] = start_Col[1];//只有是1区域的才可以将列号存储到右线里
//            }
//        }
//        else if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)//如果右线发现1区域（道路）
//        {
//            Col_Right[i] = start_Col[1];//第一次之后，剩余时候发现1区域就保持左线往右road_width
//        }
//        else
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 1 && start_Col[1]>=1)
//            {
//                start_Col[1] = start_Col[1] - 1;
//                cnt_temp = cnt_temp + 1;
//            }//则右线持续向左扫描直到不再是255区域（背景）
//            if (cnt_temp>2*road_width || start_Col[1] - start_Col[0] > 2*road_width)
//            {
//                break;
//            }
//        }

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
//    int Conv_Core[2][3][3] = {{{1,1,-1},{1,1,-1},{-1,-1,-1}},{{-1,1,1},{-1,1,1},{-1,-1,-1}}};
//    //Conv_Core(1).core = [1 1 -1
//    //                         1 1 -1
//    //                        -1 -1 -1];
//    //Conv_Core(2).core = [-1 1 1
//    //                         -1 1 1
//    //                        -1 -1 -1];
//
//    int Conv_Score_max[2] = {-9,-9};
    int Conv_Score_max_i[2] = {0,0};
    int Conv_Score_max_j[2] = {0,width_Inverse_Perspective};
//    for (int i=Search_Range[ROW][BEGIN]+0.2*Search_Range[ROW][LINES]+1;i<Search_Range[ROW][BEGIN]+0.6*Search_Range[ROW][LINES]-1-1;i++)//从第一行开始逐行扫描
//    {
//        for (int j=Search_Range[COL][BEGIN]+0.2*Search_Range[COL][LINES]+1;j<Search_Range[COL][BEGIN]+0.8*Search_Range[COL][LINES]-1-1;j++)
//        {
//            // 对于每个中心点(i,j)，计算2类卷积值，分别取最大值保留
//            int Conv_Score[2] = {0,0};//存储2类卷积结果
//            int flag = 1;//卷积合不合法
//            for (int k=0;k<2;k++)
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
//            if (Conv_Score[0] > Conv_Score_max[0] && j<width_Inverse_Perspective*0.5 && j>width_Inverse_Perspective/4)
//            {
//                Conv_Score_max[0] = Conv_Score[0];
//                Conv_Score_max_i[0] = i;
//                Conv_Score_max_j[0] = j;
//            }
//            else if (Conv_Score[0] == Conv_Score_max[0] && j<width_Inverse_Perspective*0.5 && j>width_Inverse_Perspective/4 && j>= Conv_Score_max_j[0])
//            {
//                Conv_Score_max[0] = Conv_Score[0];
//                Conv_Score_max_i[0] = i;
//                Conv_Score_max_j[0] = j;
//            }
//            if (Conv_Score[1] > Conv_Score_max[1] && j>width_Inverse_Perspective*0.5 && j<width_Inverse_Perspective*3/4)
//            {
//                Conv_Score_max[1] = Conv_Score[1];
//                Conv_Score_max_i[1] = i;
//                Conv_Score_max_j[1] = j;
//            }
//            else if (Conv_Score[1] == Conv_Score_max[1] && j>width_Inverse_Perspective*0.5 && j<width_Inverse_Perspective*3/4 && j<= Conv_Score_max_j[1])
//            {
//                Conv_Score_max[1] = Conv_Score[1];
//                Conv_Score_max_i[1] = i;
//                Conv_Score_max_j[1] = j;
//            }
//        }
//    }

    Find_Second_Dot(0);
    Conv_Score_max_i[0] = second_Dot[0];
    Conv_Score_max_j[0] = second_Dot[1];
    Find_Second_Dot(1);
    Conv_Score_max_i[1] = second_Dot[0];
    Conv_Score_max_j[1] = second_Dot[1];
    if (Conv_Score_max_i[0]==-2||Conv_Score_max_i[1]==-2 ) {
        DrawCenterLinewithConfig(0);
        return;
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
//
//uint8 Check_RightCircle(void)
//{
//    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
//    int Conv_Core[1][3][3] = {{{1,1,1},{1,1,-1},{1,-1,-1}}};
//    //Conv_Core(1).core = [1 1 1
////                         1 1 -1
////                         1 -1 -1];
//
//
//    int Conv_Score_max[1] = {-9};
//    int Conv_Score_max_i[1] = {0};
//    int Conv_Score_max_j[1] = {0};
//    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
//    {
//        for (int j=1;j<width_Inverse_Perspective-1;j++)
//        {
//            // 对于每个中心点(i,j)，计算1类卷积值，分别取最大值保留
//            int Conv_Score[1] = {0};//存储2类卷积结果
//            int flag = 1;//卷积合不合法
//            for (int k=0;k<1;k++)
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
//            if (Conv_Score[0] > Conv_Score_max[0] && i<height_Inverse_Perspective*0.7 && i>height_Inverse_Perspective*0.3 && j>width_Inverse_Perspective*0.5)
//            {
//                Conv_Score_max[0] = Conv_Score[0];
//                Conv_Score_max_i[0] = i;
//                Conv_Score_max_j[0] = j;
//            }
//        }
//    }
//
//    if (Conv_Score_max[0] >= 9)
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}
//
//uint8 Check_LeftCircle(void)
//{
//    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
//    int Conv_Core[1][3][3] = {{{1,1,1},{-1,1,1},{-1,-1,1}}};
//    //Conv_Core(1).core = [1 1 1
////                        -1 1 1
////                       -1 -1 1];
//
//
//    int Conv_Score_max[1] = {-9};
//    int Conv_Score_max_i[1] = {0};
//    int Conv_Score_max_j[1] = {0};
//    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
//    {
//        for (int j=1;j<width_Inverse_Perspective-1;j++)
//        {
//            // 对于每个中心点(i,j)，计算1类卷积值，分别取最大值保留
//            int Conv_Score[1] = {0};//存储2类卷积结果
//            int flag = 1;//卷积合不合法
//            for (int k=0;k<1;k++)
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
//            if (Conv_Score[0] > Conv_Score_max[0] && i<height_Inverse_Perspective*0.7  && i>height_Inverse_Perspective*0.3 && j<width_Inverse_Perspective*0.5)
//            {
//                Conv_Score_max[0] = Conv_Score[0];
//                Conv_Score_max_i[0] = i;
//                Conv_Score_max_j[0] = j;
//            }
//        }
//    }
//
//    if (Conv_Score_max[0] >= 9)
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}

//
//uint8 Check_ThreeRoads(void)
//{
//    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
//    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
//    int check_lines = 2;
//    uint8 result = 0;
//    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
//    {
//        if (check_lines==0)
//        {
//            break;
//        }
//        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
//        {
//            start_Row = start_Row - 1;
//            continue;
//        }
//        uint8 black_cnt = 0;
//        uint8 black_cnt_limit = 3;
//        while(mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 255)
//        {
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]]==0)
//            {
//                black_cnt++;
//            }
//            start_Col[0]--;
//        }
//        if (black_cnt>black_cnt_limit)
//        {
//            break;
//        }
//        while(mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 255)
//        {
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]]==0)
//            {
//                black_cnt++;
//            }
//            start_Col[1]++;
//        }
//        if (black_cnt>black_cnt_limit)
//        {
//            break;
//        }
//        check_lines--;
//        if (check_lines==0)
//        {
//            result=1;
//            break;
//        }
//    }
//    return result;
//}

uint8 Check_ThreeRoads_New(void)
{
    uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
    int8 direction = 1;
    int8 cnt = 0;
    int8 last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0)
        {
            water_i++;
            cnt = 0;
        }
        else if ((water_j+direction)<0||(water_j+direction)>(Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1))
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==1)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]!=0 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_down = last_angle;
    uint8 last_water_i = water_i;

    water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
    direction = 1;
    cnt = 0;
    last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0 && water_i!=last_water_i-ThreeRoads_lines[0]+1)
        {
            water_i++;
            cnt = 0;
        }
        else if ((water_j+direction)<0||(water_j+direction)>(Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1))
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==1)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]!=0 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_up = last_angle;


    water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
    direction = 1;
    cnt = 0;
    last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0 && water_i!=last_water_i-ThreeRoads_lines[1]+1)
        {
            water_i++;
            cnt = 0;
        }
        else if ((water_j+direction)<0||(water_j+direction)>(Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1))
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==1)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]!=0 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_upup = last_angle;

    float para[4]={-0.00016216f,-1.3793f,0.5517f,11.1404f};//{-0.0249f,-1.9503f,0.7702f,16.5409f};//{-0.4f,-1.2f,0.32f,15.64f};

    if ((last_angle_down*para[0]+last_angle_up*para[1]+last_angle_upup*para[2]+para[3])>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}



uint8 Check_RightCircle_New(void)
{
    uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.8*Search_Range[COL][LINES];
    int8 direction = -1;
    int8 cnt = 0;
    int8 last_angle = 1;

    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0)
        {
            water_i++;
            cnt = 0;
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==2)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==1 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_down = last_angle;
    uint8 last_water_i = water_i;
    water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.8*Search_Range[COL][LINES];
    direction = -1;
    cnt = 0;
    last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0 && water_i!=last_water_i-Circle_lines+1)
        {
            water_i++;
            cnt = 0;
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==2)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==1 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_up = last_angle;

    if ((last_angle_up - last_angle_down) >= Circle_d_target[0] && (last_angle_up - last_angle_down) <= Circle_d_target[1]
                                                                                                                        && last_angle_down >= Circle_target_down[0] && last_angle_down <= Circle_target_down[1]
                                                                                                                                                                                                             && last_angle_up >= Circle_target_up[0] && last_angle_up <= Circle_target_up[1])
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8 Check_LeftCircle_New(void)
{
    uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.2*Search_Range[COL][LINES];
    int8 direction = 1;
    int8 cnt = 0;
    int8 last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0)
        {
            water_i++;
            cnt = 0;
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==1)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==1 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_down = last_angle;
    uint8 last_water_i = water_i;
    water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.2*Search_Range[COL][LINES];
    direction = 1;
    cnt = 0;
    last_angle = 1;


    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0 && water_i!=last_water_i-Circle_lines+1)
        {
            water_i++;
            cnt = 0;
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==1)
            {
                last_angle++;
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==1 || cnt<=1)
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
        else
        {
            return 0;
        }
    }

    last_angle_up = last_angle;

    if ((last_angle_up - last_angle_down) >= Circle_d_target[0] && (last_angle_up - last_angle_down) <= Circle_d_target[1]
                                                                                                                        && last_angle_down >= Circle_target_down[0] && last_angle_down <= Circle_target_down[1]
                                                                                                                                                                                                             && last_angle_up >= Circle_target_up[0] && last_angle_up <= Circle_target_up[1])
    {
        return 1;
    }
    else
    {
        return 0;
    }

}



void Find_First_Dot(int mode)
{
    int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
    first_Dot[0] = -2;
    first_Dot[1] = -2;
    if (mode == 1)
    {
        for (int i=0;i<(Search_Range[COL][BEGIN]+Search_Range[COL][LINES] -1 - dot_j);i++)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i][dot_j+i] == 1 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i][dot_j+i+1] == 0)
            {
                first_Dot[0] = dot_i;
                first_Dot[1] = dot_j+i+1;
                break;
            }
        }
    }
    else if (mode == 0)
    {
        for (int i=0;i<dot_j-Search_Range[COL][BEGIN];i++)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i][dot_j-i] == 1 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i][dot_j-i-1] == 0)
            {
                first_Dot[0] = dot_i;
                first_Dot[1] = dot_j-i-1;
                break;
            }
        }
    }
    else if (mode == 2)
    {
        int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
        int flag=0;
        for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
        {
            for (int i=0;i<dot_j-Search_Range[COL][BEGIN];i++)
            {
                if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j-i] == 0 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j-i-1] == 1)
                {
                    first_Dot[0] = dot_i+j;
                    first_Dot[1] = dot_j-i-1;
                    flag=1;
                    break;
                }
            }
            if (flag==1)
            {
                break;
            }
        }
    }
    else if (mode == 3)
    {
        int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.25f*Search_Range[COL][LINES];
        for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j+1][dot_j] == 1 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j] == 0)
            {
                first_Dot[0] = dot_i+j;
                first_Dot[1] = dot_j;
                break;
            }
        }
    }
}

void Find_Second_Dot(int mode)
{
    second_Dot[0] = -2;
    second_Dot[1] = -2;
    if (mode == 1)
    {
        uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.8*Search_Range[COL][LINES];
        int8 direction = -1;
        int8 cnt = 0;
        while(1)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j-1]==0)
            {
                water_i++;
                water_j--;
                cnt = 0;
            }
            else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+direction][water_j+direction]==0)
            {
                water_i+=direction;
                water_j+=direction;
            }
            else
            {
                direction = -direction;
                cnt++;
                if (cnt>=4 && direction == 1)
                {
                    second_Dot[0] = water_i;
                    second_Dot[1] = water_j;
                    break;
                }
            }
        }
    }
    else if (mode == 0)
    {
        uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.2*Search_Range[COL][LINES];
        int8 direction = 1;
        int8 cnt = 0;
        while(1)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j+1]==0)
            {
                water_i++;
                water_j++;
                cnt = 0;
            }
            else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i-direction][water_j+direction]==0)
            {
                water_i-=direction;
                water_j+=direction;
            }
            else
            {
                direction = -direction;
                cnt++;
                if (cnt>=4 && direction == -1)
                {
                    second_Dot[0] = water_i;
                    second_Dot[1] = water_j;
                    break;
                }
            }
        }
    }
    else if (mode == 2)
    {
        uint8 water_i = Search_Range[ROW][BEGIN]+0.05*Search_Range[ROW][LINES], water_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
        int8 direction = 1;
        int8 cnt = 0;
        int8 flag = 0;
        uint8 dot_i[2]={0,0};
        uint8 dot_j[2]={0,0};
        while(1)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0)
            {
                water_i++;
                cnt = 0;
            }
            else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
            {
                water_j+=direction;
            }
            else
            {
                direction = -direction;
                cnt++;
                if (cnt>=4 && direction == -1)
                {
                    dot_i[0] = water_i;
                    dot_j[0] = water_j;
                    if (flag==1)
                    {
                        second_Dot[0] = 0.5*(dot_i[0]+dot_i[1]);
                        second_Dot[1] = 0.5*(dot_j[0]+dot_j[1]);
                        break;
                    }
                    else
                    {
                        flag=1;
                    }
                }
                if (cnt>=4 && direction == 1)
                {
                    dot_i[1] = water_i;
                    dot_j[1] = water_j;
                    if (flag==1)
                    {
                        second_Dot[0] = 0.5*(dot_i[0]+dot_i[1]);
                        second_Dot[1] = 0.5*(dot_j[0]+dot_j[1]);
                        break;
                    }
                    else
                    {
                        flag=1;
                    }
                }
            }
        }
    }
    else if (mode == 3)
    {
        int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.5f*Search_Range[COL][LINES];
        for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j+1][dot_j] == 1 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j] == 0)
            {
                second_Dot[0] = dot_i+j;
                second_Dot[1] = dot_j;
                break;
            }
        }
    }


}


//void Find_Third_Dot(int mode)
//{
//    third_Dot[0] = -2;
//    third_Dot[1] = -2;
//    if(second_Dot[0]!=-2)
//    {
//        if (mode == 1)
//        {
//            uint8 water_i = second_Dot[0], water_j = second_Dot[1];
//            uint8 water_j_temp = water_j;
//            for (int i = 0;i<(Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1-water_j_temp);i++)
//            {
//                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+1]==255)
//                {
//                    break;
//                }
//                else
//                {
//                    water_j+=1;
//                }
//            }//往右查找直到右侧为255区域
//            if (water_j!= (Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-2))
//            {
//                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 1)
//                {
//                    uint8 water_i_temp = water_i;
//                    for (int i = 0;i<water_i_temp-Search_Range[ROW][BEGIN];i++)
//                    {
//                        water_i--;
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+1]!=255)
//                        {
//                            while (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+1]!=255)
//                            {
//                                water_j++;
//                            }
//                        }//上移一格，并往右查找直到右侧为255区域
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 0)
//                        {
//                            third_Dot[0] = water_i;
//                            third_Dot[1] = water_j;
//                            break;
//                        }
//                    }
//                }
//                else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 0)
//                {
//                    uint8 water_i_temp = water_i;
//                    for (int i = 0;i<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - water_i_temp);i++)
//                    {
//                        water_i++;
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j]==255)
//                        {
//                            while (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j]==255)
//                            {
//                                water_j--;
//                            }
//                        }//下移一格，并往左查找直到右侧为255区域
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 1)
//                        {
//                            third_Dot[0] = water_i-1;
//                            third_Dot[1] = water_j;
//                            break;
//                        }
//                    }
//                }
//
//            }
//        }
//        else if (mode == 0)
//        {
//            uint8 water_i = second_Dot[0], water_j = second_Dot[1];
//            uint8 water_j_temp = water_j;
//            for (int i = 0;i<water_j_temp-Search_Range[COL][BEGIN];i++)
//            {
//                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-1]==255)
//                {
//                    break;
//                }
//                else
//                {
//                    water_j-=1;
//                }
//            }//往左查找直到左侧为255区域
//            if (water_j!= Search_Range[COL][BEGIN]+1)
//            {
//                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 1)
//                {
//                    uint8 water_i_temp = water_i;
//                    for (int i = 0;i<water_i_temp-Search_Range[ROW][BEGIN];i++)
//                    {
//                        water_i--;
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-1]!=255)
//                        {
//                            while (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-1]!=255)
//                            {
//                                water_j--;
//                            }
//                        }//上移一格，并往左查找直到左侧为255区域
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 0)
//                        {
//                            third_Dot[0] = water_i;
//                            third_Dot[1] = water_j;
//                            break;
//                        }
//                    }
//                }
//                else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 0)
//                {
//                    uint8 water_i_temp = water_i;
//                    for (int i = 0;i<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - water_i_temp);i++)
//                    {
//                        water_i++;
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j]==255)
//                        {
//                            while (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j]==255)
//                            {
//                                water_j++;
//                            }
//                        }//下移一格，并往右查找直到左侧为255区域
//                        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j] == 1)
//                        {
//                            third_Dot[0] = water_i-1;
//                            third_Dot[1] = water_j;
//                            break;
//                        }
//                    }
//                }
//
//            }
//        }
//
//    }
//    else if (mode == 2)
//    {
//        int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
//        for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
//        {
//            for (int i=0;i<dot_j-Search_Range[COL][BEGIN];i++)
//            {
//                if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j-i] == 0 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j-i-1] == 1)
//                {
//                    first_Dot[0] = dot_i+j;
//                    first_Dot[1] = dot_j-i-1;
//                    break;
//                }
//            }
//        }
//    }
//
//}

void Find_Third_Dot_New(int mode)
{
    third_Dot[0] = -2;
    third_Dot[1] = -2;
    if(second_Dot[0]!=-2)
    {
        if (mode == 1)
        {
            uint8 water_i = second_Dot[0]+5, water_j = second_Dot[1];
            uint8 water_j_temp = water_j;
            for (int i = 0;i<(Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1-water_j_temp);i++)
            {
                for (int j = 0;j<water_i-Search_Range[ROW][BEGIN];j++)
                {
                    if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i-1][water_j]==1)
                    {
                        water_i-=1;
                    }
                    else
                    {
                        break;
                    }
                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+1]==1)
                {
                    water_j+=1;
//                    if (water_j>=0.94f*Search_Range[COL][LINES]+Search_Range[COL][BEGIN])
//                    {
//                        third_Dot[0] = water_i-1;
//                        third_Dot[1] = water_j;
//                        break;
//                    }
                }
                else
                {
                    third_Dot[0] = water_i-1;
                    if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-3]==1)
                    {
                        third_Dot[1] = water_j-3;
                    }
                    else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-2]==1)
                    {
                        third_Dot[1] = water_j-2;
                    }
                    else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-1]==1)
                    {
                        third_Dot[1] = water_j-1;
                    }
                    else
                    {
                        third_Dot[1] = water_j;
                    }
                    break;
                }
            }
//            third_Dot[0] = water_i-1;
//            third_Dot[1] = water_j;
        }
        else if (mode == 0)
        {
            uint8 water_i = second_Dot[0]+5, water_j = second_Dot[1];
            uint8 water_j_temp = water_j;
            for (int i = 0;i<water_j_temp-Search_Range[COL][BEGIN];i++)
            {
                for (int j = 0;j<water_i-Search_Range[ROW][BEGIN];j++)
                {
                    if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i-1][water_j]==1)
                    {
                        water_i-=1;
                    }
                    else
                    {
                        break;
                    }
                }
                if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j-1]==1)
                {
                    water_j-=1;
//                    if (water_j<=(1-0.94f)*Search_Range[COL][LINES]+Search_Range[COL][BEGIN])
//                    {
//                        third_Dot[0] = water_i-1;
//                        third_Dot[1] = water_j;
//                        break;
//                    }
                }
                else
                {
                    third_Dot[0] = water_i-1;
                    if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+3]==1)
                    {
                        third_Dot[1] = water_j+3;
                    }
                    else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+2]==1)
                    {
                        third_Dot[1] = water_j+2;
                    }
                    else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+1]==1)
                    {
                        third_Dot[1] = water_j+1;
                    }
                    else
                    {
                        third_Dot[1] = water_j;
                    }
                    break;
                }
            }
//            third_Dot[0] = water_i-1;
//            third_Dot[1] = water_j;
        }
        else if (mode == 2)
        {
            int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.5*Search_Range[COL][LINES];
            int flag=0;
            for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
            {
                for (int i=0;i<(Search_Range[COL][BEGIN]+Search_Range[COL][LINES] - 1 - dot_j);i++)
                {
                    if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j+i] == 0 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j+i+1] == 1)
                    {
                        third_Dot[0] = dot_i+j;
                        third_Dot[1] = dot_j+i+1;
                        flag=1;
                        break;
                    }
                }
                if (flag==1)
                {
                    break;
                }
            }
        }
        else if (mode == 3)
        {
            int dot_i = Search_Range[ROW][BEGIN], dot_j = Search_Range[COL][BEGIN]+0.75f*Search_Range[COL][LINES];
            for (int j=0;j<(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES] - 1 - dot_i);j++)
            {
                if (mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j+1][dot_j] == 1 && mt9v03x_image_cutted_thresholding_inversePerspective[dot_i+j][dot_j] == 0)
                {
                    third_Dot[0] = dot_i+j;
                    third_Dot[1] = dot_j;
                    break;
                }
            }
        }
    }


}


uint8 Check_LeftP(void)
{
    Find_First_Dot(0);
    Find_Second_Dot(0);
    Find_Third_Dot_New(0);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int a_b_y = first_Dot[1] - second_Dot[1];
        int c_b_x = third_Dot[0] - second_Dot[0];
        int c_b_y = third_Dot[1] - second_Dot[1];
        int ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
        float dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
        float dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
        float cosValue = ab_mul_cb / (dist_ab * dist_cd);

        arccosValue = (3.1415926/2 - cosValue - 1.0f/6.0f * cosValue*cosValue*cosValue)/3.1415926*180;

        if (arccosValue > 85 && arccosValue < 100)
        {
            if (second_Dot[1]-third_Dot[1]>=2)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
}
uint8 Check_RightP(void)
{
    Find_First_Dot(1);
    Find_Second_Dot(1);
    Find_Third_Dot_New(1);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int a_b_y = first_Dot[1] - second_Dot[1];
        int c_b_x = third_Dot[0] - second_Dot[0];
        int c_b_y = third_Dot[1] - second_Dot[1];
        int ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
        float dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
        float dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
        float cosValue = ab_mul_cb / (dist_ab * dist_cd);

        arccosValue = (3.1415926/2 - cosValue - 1.0f/6.0f * cosValue*cosValue*cosValue)/3.1415926*180;

        if (arccosValue > 85 && arccosValue < 100)
        {
            if (third_Dot[1]-second_Dot[1]>=2)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
}

uint8 Check_LeftCircle_New2(void)
{
    Find_First_Dot(0);
    Find_Second_Dot(0);
    Find_Third_Dot_New(0);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int a_b_y = first_Dot[1] - second_Dot[1];
        int c_b_x = third_Dot[0] - second_Dot[0];
        int c_b_y = third_Dot[1] - second_Dot[1];
        int ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
        float dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
        float dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
        float cosValue = ab_mul_cb / (dist_ab * dist_cd);

        arccosValue = (3.1415926/2 - cosValue - 1.0f/6.0f * cosValue*cosValue*cosValue)/3.1415926*180;

        if (arccosValue < 88 && arccosValue > 0)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[(uint8)round(0.5*(third_Dot[0]+second_Dot[0]))][(uint8)round(0.5*(third_Dot[1]+second_Dot[1]))]==1)
            {
                if (second_Dot[1]-third_Dot[1]>=5)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
}

uint8 Check_RightCircle_New2(void)
{
    Find_First_Dot(1);
    Find_Second_Dot(1);
    Find_Third_Dot_New(1);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int a_b_y = first_Dot[1] - second_Dot[1];
        int c_b_x = third_Dot[0] - second_Dot[0];
        int c_b_y = third_Dot[1] - second_Dot[1];
        int ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
        float dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
        float dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
        float cosValue = ab_mul_cb / (dist_ab * dist_cd);

        arccosValue = (3.1415926/2 - cosValue - 1.0f/6.0f * cosValue*cosValue*cosValue)/3.1415926*180;

        if (arccosValue < 88 && arccosValue > 0)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[(uint8)round(0.5*(third_Dot[0]+second_Dot[0]))][(uint8)round(0.5*(third_Dot[1]+second_Dot[1]))]==1)
            {
                if (third_Dot[1]-second_Dot[1]>=5)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
}


uint8 Check_ThreeRoad_New2(void)
{
    Find_First_Dot(2);
    Find_Second_Dot(2);
    Find_Third_Dot_New(2);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int a_b_y = first_Dot[1] - second_Dot[1];
        int c_b_x = third_Dot[0] - second_Dot[0];
        int c_b_y = third_Dot[1] - second_Dot[1];
        int ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
        float dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
        float dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
        float cosValue = ab_mul_cb / (dist_ab * dist_cd);

        arccosValue = (3.1415926/2 - cosValue - 1.0f/6.0f * cosValue*cosValue*cosValue)/3.1415926*180;

        if (arccosValue > 110 && arccosValue < 130){
            return 1;
        }
        else{
            return 0;
        }
    }
}


uint8 Check_Left_for_RightCircle(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
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
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Left;
    int d_Col_Left_Num=0;

    int i;
    for (i=height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Left[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
            {
                d_Col_Left_Num = height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Left_Num<=0.7f*0.5f*Search_Range[ROW][LINES])
    {
        return 0;
    }
    return 1;
}
uint8 Check_Right_for_RightCircle(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
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
            }
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Right;
    int d_Col_Right_Num=0;

    int i;
    for (i=height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=0.7f*0.5f*Search_Range[ROW][LINES])
    {
        return 0;
    }
    return 1;
}



uint8 Check_RightCircle_New3(void)
{
    return (Check_Right_for_RightCircle()||Check_Left_for_RightCircle());
//    return (Check_Right_Straight_ForRoad(2, -2, 0.4)&&Check_Left_Straight_ForRoad(2, -2, 0.4));
}


uint8 Check_Left_for_LeftCircle(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
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
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Left;
    int d_Col_Left_Num=0;

    int i;
    for (i=height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Left[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
            {
                d_Col_Left_Num = height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Left_Num<=0.7f*0.5f*Search_Range[ROW][LINES])
    {
        return 0;
    }
    return 1;
}
uint8 Check_Right_for_LeftCircle(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
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
            }
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Right;
    int d_Col_Right_Num=0;

    int i;
    for (i=height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = height_Inverse_Perspective-(Search_Range[ROW][BEGIN]+(1.0f-0.5f)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=0.7f*0.5f*Search_Range[ROW][LINES])
    {
        return 0;
    }
    return 1;
}

uint8 Check_LeftCircle_New3(void)
{
    return (Check_Right_for_LeftCircle()||Check_Left_for_LeftCircle());
   // return (Check_Right_Straight_ForRoad(2, -2, 0.4)&&Check_Left_Straight_ForRoad(2, -2, 0.4));
}
void Compensate_ColCenter(void)
{
    int i;
    float last_center[2]={0,0};
    for (i=0;i<search_Lines;i++) //寻找从下往上的中线最底端
    {
        if (Col_Center[i] != -2)
        {
            last_center[0] = Col_Center[i];//j
            last_center[1] = (float)i;//i
            break;
        }
    }

    //存储底部中点坐标

    float buttom_center[2]={0,0};
    for (i=0;i<search_Lines;i++) //寻找视野底部
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[search_Lines-1-i][width_Inverse_Perspective/2] != 255)
        {
            buttom_center[0] = (float)(width_Inverse_Perspective/2);//j
            buttom_center[1] = (float)i;//i
            break;
        }
    }

    int firsti = i;

    if(last_center[1] <= buttom_center[1]+1 )
    {
        return;
    }
    else
    {
        //补全拐点间中线和底部中线之间的连线
        float k = (last_center[0]  - buttom_center[0])/(last_center[1]  - buttom_center[1]);

        for (int i=0;i<(last_center[1]-buttom_center[1]);i++)
        {
            Col_Center[i+firsti] = buttom_center[0] + k*i;
        }
    }


}

uint8 Check_TRoad(uint8 mode,float pos)
{
    uint8 water_i = Search_Range[ROW][BEGIN]+(1-pos)*Search_Range[ROW][LINES];
    if (mode==1)
    {
        uint8 cnt=0;
        for (int j=Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
        {
            if(mt9v03x_image_cutted_thresholding_inversePerspective[water_i][j]==0)
            {
                cnt++;
                if (cnt>=3)
                {
                    return 0;
                }
            }
        }
        return 1;
    }
    else if (mode==0)
    {
        uint8 cnt=0;
        for (int j=Search_Range[COL][BEGIN];j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];j++)
        {
            if(mt9v03x_image_cutted_thresholding_inversePerspective[water_i][j]==1)
            {
                cnt++;
                if (cnt>=3)
                {
                    return 0;
                }
            }
        }
        return 1;
    }

}

uint8 Check_Up_Straight(int8 max_d_Row_Up, int8 min_d_Row_Up)
{
    int start_Col = 0;
    int start_Row = height_Inverse_Perspective*0.8;
    int8 Row_Up[width_Inverse_Perspective_Max]={0};
    for (int i=0;i<width_Inverse_Perspective;i++)
    {
        Row_Up[i] = -2;
    }
    for (int i=0;i<width_Inverse_Perspective;i++)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col] == 255)
        {
            start_Col = start_Col + 1;
            continue;
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col] == 1) //如果左线发现1区域（道路）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col] == 1 && start_Row>=1)
            {
                start_Row = start_Row - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>2*road_width)
            {
                break;
            }
            start_Row = start_Row + 1;//则左线持续向左扫描直到不再是1区域（道路），有可能是0或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row-1][start_Col] == 0)//查看此时是否是0区域（背景）
            {
                Row_Up[i] = start_Row;//只有是0区域的才可以将列号存储到左线里
            }
        }
        else//如果左线发现0区域（背景）
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col] == 0 && start_Row<=height_Inverse_Perspective-2)
            {
                start_Row = start_Row + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>2*road_width)
            {
                break;
            }//则左线持续向右扫描直到不再是0区域（背景），有可能是1或255区域
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col] == 1)//查看此时是否是1区域（道路）
            {
                Row_Up[i] = start_Row;//只有是1区域的才可以将列号存储到左线里
            }
        }

        start_Col = start_Col + 1; //左右线扫描完毕，标记行进入上一行，给下一次左右线扫描做准备
    }

    start_Col = Search_Range[COL][BEGIN];
    int d_Row_Up;
    int d_Row_Up_Num=0;

    int i;
    for (i=Search_Range[COL][BEGIN];i<Search_Range[COL][BEGIN]+Search_Range[COL][LINES];i++)
    {
        if (Row_Up[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[Row_Up[i]-1][start_Col] == 0)
            {
                d_Row_Up_Num = Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-i-1;
                break;
            }
        }
        start_Col = start_Col + 1;
    }
    if (d_Row_Up_Num<=0.1f*Search_Range[COL][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Row_Up_Num;j++)
    {
        d_Row_Up = Row_Up[i+1]-Row_Up[i];
        i++;
        if (d_Row_Up < min_d_Row_Up || d_Row_Up > max_d_Row_Up)
        {
            return 0;
        }
    }
    return 1;
}

uint8 Check_Left_Straight(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
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
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Left;
    int d_Col_Left_Num=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Left[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
            {
                d_Col_Left_Num = height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Left_Num<=0.7f*ratio*Search_Range[ROW][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Col_Left_Num;j++)
    {
        d_Col_Left = Col_Left[i+1]-Col_Left[i];
        i++;
        if (d_Col_Left < min_d_Col_Left || d_Col_Left > max_d_Col_Left)
        {
            return 0;
        }
    }
    return 1;
}

uint8 Check_Right_Straight(int8 max_d_Col_Right, int8 min_d_Col_Right, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
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
            }
            if (cnt_temp>2*road_width)
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
            if (cnt_temp>2*road_width)
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
    }

    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
    int d_Col_Right;
    int d_Col_Right_Num=0;

    int i;
    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES])-i-1;
                break;
            }
            //            else
            //            {
            //                return 0;
            //            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=0.7f*ratio*Search_Range[ROW][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Col_Right_Num;j++)
    {
        d_Col_Right = Col_Right[i+1]-Col_Right[i];
        i++;
        if (d_Col_Right < min_d_Col_Right || d_Col_Right > max_d_Col_Right)
        {
            return 0;
        }
    }
    return 1;
}

uint8 Check_TRoad_New(void)
{
    Find_First_Dot(3);
    Find_Second_Dot(3);
    Find_Third_Dot_New(3);

    if ( (first_Dot[0]==-2 )||(second_Dot[0]==-2 )||(third_Dot[0]==-2 )) {
        return 0;
    }

    else{
        int a_b_x = first_Dot[0] - second_Dot[0];
        int c_b_x = third_Dot[0] - second_Dot[0];


        if ((a_b_x+c_b_x)>=-2 && (a_b_x+c_b_x)<=2)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

float max(float a,float b)
{
    if (a>=b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

float min(float a,float b)
{
    if (a<b)
    {
        return a;
    }
    else
    {
        return b;
    }
}


float Filter_Col_Left(uint8 flag,float value,float ratio)
{
    static float last_value=0;
    float new_value;
    if (flag==0)
    {
        last_value=value;
        return value;
    }
    else if(flag==1)
    {
        new_value=ratio*value+(1-ratio)*last_value;
        last_value = new_value;
        return new_value;
    }
}

float Filter_Col_Right(uint8 flag,float value,float ratio)
{
    static float last_value=0;
    float new_value;
    if (flag==0)
    {
        last_value=value;
        return value;
    }
    else if(flag==1)
    {
        new_value=ratio*value+(1-ratio)*last_value;
        last_value = new_value;
        return new_value;
    }
}

uint8 Check_Left_All_Road(float ratio,int zero_limit)
{
    int cnt=0;
    for (int i=0;i<height_Inverse_Perspective;i++)
    {
        uint8 flag=0;
        for (int j=0;j<width_Inverse_Perspective-1;j++)
        {
            int k_limit = zero_limit<(width_Inverse_Perspective-1-j)?zero_limit:(width_Inverse_Perspective-1-j);
            for (int k=1;k<=k_limit;k++)
            {
                if (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==255 && mt9v03x_image_cutted_thresholding_inversePerspective[i][j+k]==1)
                {
                    cnt++;
                    flag=1;
                    break;
                }
            }
            if (flag==1)
            {
                break;
            }
        }
    }
    float Real_Ratio = cnt*1.0f/height_Inverse_Perspective;
    if (Real_Ratio>=ratio)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8 Check_Right_All_Road(float ratio,int zero_limit)
{
    int cnt=0;
    for (int i=0;i<height_Inverse_Perspective;i++)
    {
        uint8 flag=0;
        for (int j=width_Inverse_Perspective-2;j>=0;j--)
        {
            int k_limit = zero_limit<(width_Inverse_Perspective-1-j)?zero_limit:(width_Inverse_Perspective-1-j);
            for (int k=1;k<=k_limit;k++)
            {
                if (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==1 && mt9v03x_image_cutted_thresholding_inversePerspective[i][j+k]==255)
                {
                    cnt++;
                    flag=1;
                    break;
                }
            }
            if (flag==1)
            {
                break;
            }
        }
    }
    float Real_Ratio = cnt*1.0f/height_Inverse_Perspective;
    if (Real_Ratio>=ratio)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8 Select_Left_Unknown_or_Right(int dot_num)
{
    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    search_Lines = height_Inverse_Perspective;//一共要扫描多少行，最大是图片宽
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center_Backup[i]=Col_Center[i];
        Col_Center[i] = -2;
    }
    DrawCenterLinewithConfig_RightBased(0);
    Right_Straight_Score = Get_Straight_Score(dot_num);

    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    search_Lines = height_Inverse_Perspective;//一共要扫描多少行，最大是图片宽
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = -2;
    }
    DrawCenterLinewithConfig_LeftBased(0);
    Left_Straight_Score = Get_Straight_Score(dot_num);

    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    search_Lines = height_Inverse_Perspective;//一共要扫描多少行，最大是图片宽
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = -2;
    }
    DrawCenterLinewithConfig(0);
    Unknown_Straight_Score = Get_Straight_Score(dot_num);

    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = Col_Center_Backup[i];
    }
}

float Get_Straight_Score(int dot_num)
{
    uint8 area_divide = dot_num-1;
    uint8 Row_Index[MAX_Row_Index];
    float K[MAX_Row_Index];
    Row_Index[0] = (uint8)round(height_Inverse_Perspective*0.1f);
    Row_Index[dot_num-1] = (uint8)round(height_Inverse_Perspective*0.9f);
    float Col_Center_Init = width_Inverse_Perspective/2;
    for (int i=1;i<=dot_num-2;i++)
    {
        Row_Index[i] = (uint8)round(height_Inverse_Perspective*1.0f/area_divide*i);
    }
    uint8 cnt=0;
    float K_Ave=0;
    for (int i=0;i<=dot_num-1;i++)
    {
        K[i]=-2;
        if (Col_Center[Row_Index[i]]!=-2)
        {
            K[i] = (Col_Center[Row_Index[i]]-Col_Center_Init)/Row_Index[i];
            cnt++;
        }
    }
    if (cnt==0)
    {
        return 0;
    }
    for (int i=0;i<=dot_num-1;i++)
    {
        if (K[i]!=-2)
        {
            K_Ave+=K[i];
        }
    }
    K_Ave = K_Ave/cnt;
    float score=0;
    for (int i=0;i<=dot_num-1;i++)
    {
        if (K[i]!=-2)
        {
            score+=1-fabsf(K[i]-K_Ave);
        }
    }
    score = score*6.0f/dot_num;
    return score;

}

void Check_Zebra(float pos)
{
    uint8 white2black_cnt=0;
    uint8 i = (1.0f-pos)*Y_WIDTH_CAMERA;
    int Left_dot=-2;
    int Right_dot=-2;
    if (mt9v03x_image[i][0]>thresholding_Value)
    {
        white2black_cnt++;
        if (Left_dot==-2)
        {
            Left_dot = 0;
        }
        Right_dot = 0;
    }
    for (int j=0;j<X_WIDTH_CAMERA-2;j++)
    {
        if (mt9v03x_image[i][j]>thresholding_Value && mt9v03x_image[i][j+1]<=thresholding_Value)
        {
            white2black_cnt++;
            if (Left_dot==-2)
            {
                Left_dot = j;
            }
            Right_dot = j;
        }
        if (mt9v03x_image[i][j]<=thresholding_Value && mt9v03x_image[i][j+1]>thresholding_Value)
        {
            white2black_cnt++;
            if (Left_dot==-2)
            {
                Left_dot = j;
            }
            Right_dot = j;
        }
    }
    if (mt9v03x_image[i][X_WIDTH_CAMERA-1]>thresholding_Value)
    {
        white2black_cnt++;
        if (Left_dot==-2)
        {
            Left_dot = X_WIDTH_CAMERA-1;
        }
        Right_dot = X_WIDTH_CAMERA-1;
    }

    static uint8 last_white2black_cnt=0;
    static uint8 cnt_filter=0;
    int valid_flag = 0;
    if (last_white2black_cnt==white2black_cnt)
    {
        cnt_filter++;
        if (cnt_filter>=1)
        {
            valid_flag=1;
        }
    }
    else
    {
        cnt_filter=0;
        last_white2black_cnt = white2black_cnt;
    }

    valid_flag = 1;//不滤波

    if (valid_flag==1)
    {
        White2Black_cnt = white2black_cnt;

        float direction = 0.5f*(Right_dot+Left_dot);
        if (direction>0 && direction<(X_WIDTH_CAMERA-1)*0.5f)
        {
            zebra_direction = -1;
        }
        else if (direction>(X_WIDTH_CAMERA-1)*0.5f && direction<(X_WIDTH_CAMERA-1))
        {
            zebra_direction = 1;
        }
        else
        {
            zebra_direction = 0;
        }
    }
}



uint8 Check_Fake_Slope(int max){
    int i;
    int cnt=0;
    for (i = height_Inverse_Perspective  - (int)( Lazer_Data / (ratioOfPixelToHG * 30.0f));i < height_Inverse_Perspective;i++ ){
        if(mt9v03x_image_cutted_thresholding_inversePerspective[i][width_Inverse_Perspective/2] == 0){
            cnt++;
            if(cnt > max){
                return 0;
            }
        }
    }
    return 1;
}

uint8 Check_Fake_Zebra(int max){
    int i;
    int cnt=0;
    for (i = (1- 0.6)*height_Inverse_Perspective  ;i < height_Inverse_Perspective;i++ ){
        if(mt9v03x_image_cutted_thresholding_inversePerspective[i][width_Inverse_Perspective/2] == 0){
            cnt++;
            if(cnt > max){
                return 0;
            }
        }
    }
    return 1;
}




int set_later_white(int i,int j,uint8 valid)
{
    int k=1;
    for (k=1;k<width_Inverse_Perspective-j;k++)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[i][j+k] == 1)
        {
            valid_table[i][j+k] = valid;
            continue;
        }
        else
        {
            return (j+k-1);
        }
    }
    return (j+k-1);
}

void set_before_white(int i,int j,uint8 valid)
{
    int k=1;
    for (k=1;k<=j;k++)
    {
        if (valid_table[i][j-k] == 2)
        {
            valid_table[i][j+k] = valid;
            continue;
        }
        else
        {
            return;
        }
    }
    return;
}

void Find_Valid_White(void)
{
    //valid_table 0:无效 1:有效 2:备选项
    for (int i=height_Inverse_Perspective-1;i>=0;i--)
    {
        for (int j=0;j<width_Inverse_Perspective;j++)
        {
            valid_table[i][j]=0;//初始化为无效
            //遇到黑色或255，前面所有备选项均无效
            if (mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==255||mt9v03x_image_cutted_thresholding_inversePerspective[i][j]==0)
            {
                valid_table[i][j] = 0;
                set_before_white(i,j,0);
                continue;
            }
            //遇到白色执行下方内容
            //最后一行所有白色均有效
            if (i==height_Inverse_Perspective-1)
            {
                valid_table[i][j] = 1;
                continue;
            }
            else
            {
                //非最后一行遇到白色，检查下方一个像素：
                    //如果下方是黑色或255，则该像素作为备选项，并检查下一个像素；
                    //如果下方是无效白色，则当前像素无效，前面所有备选项均无效、后面所有的白色均无效；
                    //如果下方是有效白色，则当前像素有效，前面所有备选项均有效、后面所有的白色均有效
                if (mt9v03x_image_cutted_thresholding_inversePerspective[i+1][j] == 0||mt9v03x_image_cutted_thresholding_inversePerspective[i+1][j] == 255)
                {
                    valid_table[i][j] = 2;
                    continue;
                }
                else if (mt9v03x_image_cutted_thresholding_inversePerspective[i+1][j] == 1 && valid_table[i+1][j]==0)
                {
                    valid_table[i][j] = 0;
                    set_before_white(i,j,0);
                    j = set_later_white(i,j,0);
                    continue;
                }
                else if (mt9v03x_image_cutted_thresholding_inversePerspective[i+1][j] == 1 && valid_table[i+1][j]==1)
                {
                    valid_table[i][j] = 1;
                    set_before_white(i,j,1);
                    j = set_later_white(i,j,1);
                    continue;
                }
            }
        }
    }
}


