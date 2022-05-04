#include "headfile.h"
#include "CAMERA.h"
#include "SEARCH.h"
#include "fastlz.h"//压缩算法
#include <stdlib.h>
#include "OLED.h"


//需要串口通信输出去，但不用传过来的变量
float Col_Center[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储中心线线的列号结果，不合法的全部为-2
int Col_Left[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储左线的列号结果，不合法的全部为-2
int Col_Right[height_Inverse_Perspective_Max] = {-2};//按从下往上的顺序存储右线的列号结果，不合法的全部为-2



//需要串口通信传过来的变量（必须配以执行变量更新的函数）


//不需要传输的其他变量
int search_Lines_Straight;//指直线检测的有效扫描行数
int search_Lines;//指Col_Center的有效扫描行数，用于遍历Col_Center

float threeRoads_RightTime = 0.15f;
float rightCircle_RightTime = 0.5f;
float rightCircle_LeftTime = 0.5f;
float rightCircle_BannedTime = 3.0f;

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
    search_Lines = height_Inverse_Perspective;//一共要扫描多少行，最大是图片宽
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
    // 对于4三岔路口可以采用，特征是靠右行驶
    else if (classification_Result == 4)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // 对于8靠右（临时使用）可以采用，特征是靠右行驶
    else if (classification_Result == 8)
    {
        DrawCenterLinewithConfig_RightBased(-0.3);
    }
    // 对于7靠左（临时使用）可以采用，特征是靠左行驶
    else if (classification_Result == 7)
    {
        DrawCenterLinewithConfig_LeftBased(-0.3);
    }
    // 对于0左弯、1右弯以及剩余还没写好的道路元素，可以采用，特征是滤波是负数，用于超前转向，以免冲出弯道
    else
    {
        DrawCenterLinewithConfig(0);
    }
}

uint8 Check_Straight(void)
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

uint8 Check_Left_Straight_New(int8 max_d_Col_Left, int8 min_d_Col_Left, float max_dd_Col_Left, float min_dd_Col_Left)
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
    int d_Col_Left[2]={0};
    int32 dd_Col_Left = 0;
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
    if (d_Col_Left_Num<=5)
    {
        return 0;
    }
    for (int j=0;j<d_Col_Left_Num;j++)
    {
        d_Col_Left[0] = d_Col_Left[1];
        d_Col_Left[1] = Col_Left[i+1]-Col_Left[i];
        i++;
        if (d_Col_Left[1] < min_d_Col_Left || d_Col_Left[1] > max_d_Col_Left)
        {
            return 0;
        }
        if (j>=1)
        {
            dd_Col_Left += d_Col_Left[1] - d_Col_Left[0];
        }
    }
    float mean = dd_Col_Left / ((d_Col_Left_Num-1)*1.0f);
    if (mean < min_dd_Col_Left || mean > max_dd_Col_Left)
    {
        return 0;
    }
    return 1;
}


uint8 Check_Right_Straight_New(int8 max_d_Col_Right, int8 min_d_Col_Right, float max_dd_Col_Right, float min_dd_Col_Right)
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
    }

    start_Row = height_Inverse_Perspective-1;
    int d_Col_Right[2]={0};
    int32 dd_Col_Right = 0;
    int d_Col_Right_Num=0;

    int i;
    for (i=0;i<search_Lines;i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = search_Lines-i-1;
                break;
            }
            else
            {
                return 0;
            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=5)
    {
        return 0;
    }
    for (int j=0;j<d_Col_Right_Num;j++)
    {
        d_Col_Right[0] = d_Col_Right[1];
        d_Col_Right[1] = Col_Right[i+1]-Col_Right[i];
        i++;
        if (d_Col_Right[1] < min_d_Col_Right || d_Col_Right[1] > max_d_Col_Right)
        {
            return 0;
        }
        if (j>=1)
        {
            dd_Col_Right += d_Col_Right[1] - d_Col_Right[0];
        }
    }
    float mean = dd_Col_Right / ((d_Col_Right_Num-1)*1.0f);
    if (mean < min_dd_Col_Right || mean > max_dd_Col_Right)
    {
        return 0;
    }
    return 1;
}


uint8 Check_Left_Straight(int8 max_d_Col_Left, int8 min_d_Col_Left)
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
    if (d_Col_Left_Num<=5)
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


uint8 Check_Right_Straight(int8 max_d_Col_Right, int8 min_d_Col_Right)
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
    }

    start_Row = height_Inverse_Perspective-1;
    int d_Col_Right;
    int d_Col_Right_Num=0;

    int i;
    for (i=0;i<search_Lines;i++)
    {
        if (Col_Right[i] !=-2)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Right[i]-1] == 1)
            {
                d_Col_Right_Num = search_Lines-i-1;
                break;
            }
            else
            {
                return 0;
            }
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=5)
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

//filter滤波系数，正数时是低通滤波，负数时相当于高通滤波
void DrawCenterLinewithConfig(float filter)
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

uint8 Check_RightCircle(void)
{
    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
    int Conv_Core[1][3][3] = {{{1,1,1},{1,1,-1},{1,-1,-1}}};
    //Conv_Core(1).core = [1 1 1
//                         1 1 -1
//                         1 -1 -1];


    int Conv_Score_max[1] = {-9};
    int Conv_Score_max_i[1] = {0};
    int Conv_Score_max_j[1] = {0};
    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
    {
        for (int j=1;j<width_Inverse_Perspective-1;j++)
        {
            // 对于每个中心点(i,j)，计算1类卷积值，分别取最大值保留
            int Conv_Score[1] = {0};//存储2类卷积结果
            int flag = 1;//卷积合不合法
            for (int k=0;k<1;k++)
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
            if (Conv_Score[0] > Conv_Score_max[0] && i<height_Inverse_Perspective*0.7 && i>height_Inverse_Perspective*0.3 && j>width_Inverse_Perspective*0.5)
            {
                Conv_Score_max[0] = Conv_Score[0];
                Conv_Score_max_i[0] = i;
                Conv_Score_max_j[0] = j;
            }
        }
    }

    if (Conv_Score_max[0] >= 9)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8 Check_LeftCircle(void)
{
    int full_Lines = height_Inverse_Perspective;//一共要从上往下扫描多少行，最大是图片宽
    int Conv_Core[1][3][3] = {{{1,1,1},{-1,1,1},{-1,-1,1}}};
    //Conv_Core(1).core = [1 1 1
//                        -1 1 1
//                       -1 -1 1];


    int Conv_Score_max[1] = {-9};
    int Conv_Score_max_i[1] = {0};
    int Conv_Score_max_j[1] = {0};
    for (int i=1;i<full_Lines-1;i++)//从第一行开始逐行扫描
    {
        for (int j=1;j<width_Inverse_Perspective-1;j++)
        {
            // 对于每个中心点(i,j)，计算1类卷积值，分别取最大值保留
            int Conv_Score[1] = {0};//存储2类卷积结果
            int flag = 1;//卷积合不合法
            for (int k=0;k<1;k++)
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
            if (Conv_Score[0] > Conv_Score_max[0] && i<height_Inverse_Perspective*0.7  && i>height_Inverse_Perspective*0.3 && j<width_Inverse_Perspective*0.5)
            {
                Conv_Score_max[0] = Conv_Score[0];
                Conv_Score_max_i[0] = i;
                Conv_Score_max_j[0] = j;
            }
        }
    }

    if (Conv_Score_max[0] >= 9)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


uint8 Check_ThreeRoads(void)
{
    int start_Row = height_Inverse_Perspective-1;//标记当前在处理哪一行，从最后一行开始
    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};//标记当前在处理哪一列，start_Col(1)指左线，start_Col(2)指右线，默认从中心两侧5像素开始
    int check_lines = 2;
    uint8 result = 0;
    for (int i=0;i<search_Lines;i++)//从最后一行开始逐行扫描，一共扫描search_Lines行
    {
        if (check_lines==0)
        {
            break;
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }
        uint8 black_cnt = 0;
        uint8 black_cnt_limit = 3;
        while(mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 255)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]]==0)
            {
                black_cnt++;
            }
            start_Col[0]--;
        }
        if (black_cnt>black_cnt_limit)
        {
            break;
        }
        while(mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 255)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]]==0)
            {
                black_cnt++;
            }
            start_Col[1]++;
        }
        if (black_cnt>black_cnt_limit)
        {
            break;
        }
        check_lines--;
        if (check_lines==0)
        {
            result=1;
            break;
        }
    }
    return result;
}

uint8 Check_ThreeRoads_New(void)
{
    uint8 water_i = 0, water_j = 0.5*width_Inverse_Perspective;
    int8 direction = 1;
    int8 cnt = 0;
    int8 last_angle = 1;
    while(1)
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i+1][water_j]==0)
        {
            water_i++;
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[water_i][water_j+direction]==0)
        {
            water_j+=direction;
            if (cnt==3)
            {
                last_angle++;
                if (last_angle>4)
                {
                    return 0;
                }
            }
        }
        else
        {
            direction = -direction;
            cnt++;
            if (cnt>=4)
            {
                break;
            }
        }
    }
    return 1;
}
