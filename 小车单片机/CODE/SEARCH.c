#include "headfile.h"
#include "CAMERA.h"
#include "SEARCH.h"
#include "fastlz.h"//ѹ���㷨
#include <stdlib.h>
#include "OLED.h"


//��Ҫ����ͨ�����ȥ�������ô������ı���
float Col_Center[height_Inverse_Perspective_Max] = {-2};//���������ϵ�˳��洢�������ߵ��кŽ�������Ϸ���ȫ��Ϊ-2
int Col_Left[height_Inverse_Perspective_Max] = {-2};//���������ϵ�˳��洢���ߵ��кŽ�������Ϸ���ȫ��Ϊ-2
int Col_Right[height_Inverse_Perspective_Max] = {-2};//���������ϵ�˳��洢���ߵ��кŽ�������Ϸ���ȫ��Ϊ-2



//��Ҫ����ͨ�Ŵ������ı�������������ִ�б������µĺ�����


//����Ҫ�������������
int road_width; //��·ʵ�ʿ��0.4m

int search_Lines_Straight;//ֱָ�߼�����Чɨ������
int search_Lines;//ָCol_Center����Чɨ�����������ڱ���Col_Center

float threeRoads_RightTime = 0.15f;
float rightCircle_RightTime = 0.5f;
float rightCircle_LeftTime = 0.2f;
float rightCircle_BannedTime = 3.0f;
float T_Time = 0.5f;

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
//11��20�Ǽ�����

int8 last_angle_down;
int8 last_angle_up;
int8 last_angle_upup;

int8 first_Dot[2];
int8 second_Dot[2];
int8 third_Dot[2];
float arccosValue;

void UART_ColCenter(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x09);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Center[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)round(Col_Center[i]));
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x09);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void UART_ColLeft(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x10);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Left[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)Col_Left[i]);
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x10);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void UART_ColRight(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x11);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        uart_putchar(DEBUG_UART, (uint8)(Col_Right[i] == -2));
        uart_putchar(DEBUG_UART, (uint8)Col_Right[i]);
    }
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x11);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void DrawCenterLine(void)
{
    road_width = (0.4/Camera_Height/ratioOfPixelToHG);
    search_Lines = height_Inverse_Perspective;//һ��Ҫɨ������У������ͼƬ��
    for (int i=0;i<height_Inverse_Perspective_Max;i++)
    {
        Col_Center[i] = -2;
    }

    // ����6ֱ�������Բ��ã��������˲��ǽϴ����������ڱ����������й���Ĳ���
    if (classification_Result == 6)
    {
        DrawCenterLinewithConfig(0.7);
    }
    // ����5ʮ��·�ڣ����Բ��ã�������ʹ�þ�����ж������յ�
    else if (classification_Result == 5)
    {
        DrawCenterLinewithConfig_CrossRoad();
    }
    // ����4����·�ڿ��Բ��ã������ǿ�����ʻ
    else if (classification_Result == 4)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // ����8���ң���ʱʹ�ã����Բ��ã������ǿ�����ʻ
    else if (classification_Result == 8)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // ����3�һ������Բ��ã������ǿ�����ʻ
    else if (classification_Result == 3)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // ����11��ֱ�߿��Բ��ã������ǿ�����ʻ
    else if (classification_Result == 11)
    {
        DrawCenterLinewithConfig_RightBased(0);
    }
    // ����2�󻷵����Բ��ã������ǿ�����ʻ
    else if (classification_Result == 2)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    // ����10��ֱ�߿��Բ��ã������ǿ�����ʻ
    else if (classification_Result == 10)
    {
        DrawCenterLinewithConfig_LeftBased(0);
    }
    // ����7������ʱʹ�ã����Բ��ã������ǿ�����ʻ
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
    // ����0���䡢1�����Լ�ʣ�໹ûд�õĵ�·Ԫ�أ����Բ��ã��������˲��Ǹ��������ڳ�ǰת�����������
    else
    {
        DrawCenterLinewithConfig(0);
    }
}

uint8 Check_Straight(float ratio)
{
    int start_Row = height_Inverse_Perspective-1;
    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};

    search_Lines_Straight=(int)(3/ratioOfPixelToHG); //��ʾֱ��ɨ�跶ΧΪ��ǰ3.0������ͷ�߶�
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


uint8 Check_Left_Straight(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
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
            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
            {
                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
    if (d_Col_Left_Num<=0.7*ratio*Search_Range[ROW][LINES])
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
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
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
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
    if (d_Col_Right_Num<=0.7*ratio*Search_Range[ROW][LINES])
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

//filter�˲�ϵ��������ʱ�ǵ�ͨ�˲�������ʱ�൱�ڸ�ͨ�˲�
void DrawCenterLinewithConfig(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ

    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
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
            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
            {
                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
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
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��

        //�����������߼���
        if (start_Col[0]>start_Col[1] || start_Col[1] - start_Col[0] > 2*road_width)
        {
            continue;
        }
        //�����߼�����4����������ߺϷ�(!=-2)��Ƿ�(==-2)�������ߺϷ�(!=-2)��Ƿ�(==-2)��
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //���ߺϷ������ߺϷ�
        {
            road_width = Col_Right[i] - Col_Left[i];
            if (Col_Center[i-1]!=-2) //�����һ��������Ҳ�Ϸ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //������һ�ε������ߡ���һ����������ֵ�����˲�������ε�������
            }
            else //�����һ�������߲��Ϸ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //�ٶ���һ�������������м䣬������һ�������ߡ���һ����������ֵ�����˲�������ε�������
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //���߷Ƿ������ߺϷ�
        {
            if (Col_Center[i-1]!=-2) //�����һ�������ߺϷ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //������һ�ε������ߡ��ٶ�����һ�������ߣ�������һ�κ����ߵľ��룩�����˲�������ε�������
            }
            else //�����һ�������߷Ƿ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //��һ��������ֱ�Ӽٶ�Ϊ���м�
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //���ߺϷ������߷Ƿ�������������
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
        else //���߷Ƿ������߷Ƿ�
        {

        }
        //�����߼������
    }
}

void DrawCenterLinewithConfig_RightBased(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ

    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1 && start_Col[1]<=width_Inverse_Perspective-2)
            {
                start_Col[1] = start_Col[1] + 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>(flag_For_Right_Circle==1?0.7*road_width:2*road_width))
            {
                break;
            }
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        start_Col[0] = start_Col[1]-road_width;//���ߵĳ�ʼɨ�������������road_width
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
        {
            Col_Left[i] = start_Col[0];//��һ��֮��ʣ��ʱ����1����ͱ�����������road_width
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0)//������߷���0���򣨱�����
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 1 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (cnt_temp>2*road_width)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] != 1 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (cnt_temp>2*road_width)
            {
                break;
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��

        //�����������߼���
        if (start_Col[0]>start_Col[1] || start_Col[1] - start_Col[0] > 2*road_width)
        {
            continue;
        }
        //�����߼�����4����������ߺϷ�(!=-2)��Ƿ�(==-2)�������ߺϷ�(!=-2)��Ƿ�(==-2)��
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //���ߺϷ������ߺϷ�
        {
            if (Col_Center[i-1]!=-2) //�����һ��������Ҳ�Ϸ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //������һ�ε������ߡ���һ����������ֵ�����˲�������ε�������
            }
            else //�����һ�������߲��Ϸ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //�ٶ���һ�������������м䣬������һ�������ߡ���һ����������ֵ�����˲�������ε�������
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //���߷Ƿ������ߺϷ�
        {
            if (Col_Center[i-1]!=-2) //�����һ�������ߺϷ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //������һ�ε������ߡ��ٶ�����һ�������ߣ�������һ�κ����ߵľ��룩�����˲�������ε�������
            }
            else //�����һ�������߷Ƿ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //��һ��������ֱ�Ӽٶ�Ϊ���м�
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //���ߺϷ������߷Ƿ�������������
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
        else //���߷Ƿ������߷Ƿ�
        {

        }
        //�����߼������
    }
}

void DrawCenterLinewithConfig_LeftBased(float filter)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ

    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255 || mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255 || start_Col[0]>start_Col[1])
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
            {
                start_Col[0] = start_Col[0] - 1;
                cnt_temp = cnt_temp + 1;
            }
            if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
            {
                break;
            }
            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
            {
                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���0���򣨱�����
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
            {
                start_Col[0] = start_Col[0] + 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (cnt_temp>(flag_For_Left_Circle==1?0.7*road_width:2*road_width))
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        start_Col[1] = start_Col[0]+road_width;//���ߵĳ�ʼɨ�������������road_width
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 1 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (cnt_temp>2*road_width)
            {
                break;
            }
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)//������߷���1���򣨵�·��
        {
            Col_Right[i] = start_Col[1];//��һ��֮��ʣ��ʱ����1����ͱ�����������road_width
        }
        else
        {
            int cnt_temp = 0;
            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] != 1 && start_Col[1]>=1)
            {
                start_Col[1] = start_Col[1] - 1;
                cnt_temp = cnt_temp + 1;
            }//�����߳�������ɨ��ֱ��������255���򣨱�����
            if (cnt_temp>2*road_width || start_Col[1] - start_Col[0] > 2*road_width)
            {
                break;
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��

        //�����������߼���
        if (start_Col[0]>start_Col[1])
        {
            continue;
        }
        //�����߼�����4����������ߺϷ�(!=-2)��Ƿ�(==-2)�������ߺϷ�(!=-2)��Ƿ�(==-2)��
        if (Col_Right[i]!=-2 && Col_Left[i]!= -2) //���ߺϷ������ߺϷ�
        {
            if (Col_Center[i-1]!=-2) //�����һ��������Ҳ�Ϸ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //������һ�ε������ߡ���һ����������ֵ�����˲�������ε�������
            }
            else //�����һ�������߲��Ϸ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*0.5*(Col_Right[i] + Col_Left[i]); //�ٶ���һ�������������м䣬������һ�������ߡ���һ����������ֵ�����˲�������ε�������
            }
        }
        else if (Col_Right[i]!=-2 && Col_Left[i]==-2) //���߷Ƿ������ߺϷ�
        {
            if (Col_Center[i-1]!=-2) //�����һ�������ߺϷ�
            {
                Col_Center[i] = filter*Col_Center[i-1]+(1-filter)*(Col_Right[i]-road_width/2.0f); //������һ�ε������ߡ��ٶ�����һ�������ߣ�������һ�κ����ߵľ��룩�����˲�������ε�������
            }
            else //�����һ�������߷Ƿ�
            {
                Col_Center[i] = filter*(width_Inverse_Perspective/2)+(1-filter)*(Col_Right[i]-road_width/2.0f); //��һ��������ֱ�Ӽٶ�Ϊ���м�
            }
        }
        else if (Col_Right[i]==-2 && Col_Left[i]!=-2) //���ߺϷ������߷Ƿ�������������
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
        else //���߷Ƿ������߷Ƿ�
        {

        }
        //�����߼������
    }
}

void DrawCenterLinewithConfig_CrossRoad(void)
{
    int full_Lines = Search_Range[ROW][LINES];//һ��Ҫ��������ɨ������У������ͼƬ��
    int Conv_Core[2][3][3] = {{{1,1,-1},{1,1,-1},{-1,-1,-1}},{{-1,1,1},{-1,1,1},{-1,-1,-1}}};
    //Conv_Core(1).core = [1 1 -1
//                         1 1 -1
//                        -1 -1 -1];
    //Conv_Core(2).core = [-1 1 1
//                         -1 1 1
//                        -1 -1 -1];

    int Conv_Score_max[2] = {-9,-9};
    int Conv_Score_max_i[2] = {0,0};
    int Conv_Score_max_j[2] = {0,width_Inverse_Perspective};
    for (int i=Search_Range[ROW][BEGIN]+1;i<Search_Range[ROW][BEGIN]+full_Lines-1;i++)//�ӵ�һ�п�ʼ����ɨ��
    {
        for (int j=Search_Range[COL][BEGIN]+1;j<Search_Range[COL][BEGIN]+Search_Range[COL][LINES]-1;j++)
        {
            // ����ÿ�����ĵ�(i,j)������2����ֵ���ֱ�ȡ���ֵ����
            int Conv_Score[2] = {0,0};//�洢2�������
            int flag = 1;//����ϲ��Ϸ�
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
            if (Conv_Score[0] > Conv_Score_max[0] && j<width_Inverse_Perspective*0.5 && j>width_Inverse_Perspective/4)
            {
                Conv_Score_max[0] = Conv_Score[0];
                Conv_Score_max_i[0] = i;
                Conv_Score_max_j[0] = j;
            }
            else if (Conv_Score[0] == Conv_Score_max[0] && j<width_Inverse_Perspective*0.5 && j>width_Inverse_Perspective/4 && j>= Conv_Score_max_j[0])
            {
                Conv_Score_max[0] = Conv_Score[0];
                Conv_Score_max_i[0] = i;
                Conv_Score_max_j[0] = j;
            }
            if (Conv_Score[1] > Conv_Score_max[1] && j>width_Inverse_Perspective*0.5 && j<width_Inverse_Perspective*3/4)
            {
                Conv_Score_max[1] = Conv_Score[1];
                Conv_Score_max_i[1] = i;
                Conv_Score_max_j[1] = j;
            }
            else if (Conv_Score[1] == Conv_Score_max[1] && j>width_Inverse_Perspective*0.5 && j<width_Inverse_Perspective*3/4 && j<= Conv_Score_max_j[1])
            {
                Conv_Score_max[1] = Conv_Score[1];
                Conv_Score_max_i[1] = i;
                Conv_Score_max_j[1] = j;
            }
        }
    }

    //�洢�ײ��е�����
    int i;
    for (i=0;i<search_Lines;i++) //Ѱ����Ұ�ײ�
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[search_Lines-1-i][width_Inverse_Perspective/2] != 255)
        {
            Col_Center[i] = (float)(width_Inverse_Perspective/2);
            break;
        }
    }


    int firsti = i;

    //��ȫ�յ�����ߺ͵ײ�����֮�������
    float k = ((Conv_Score_max_j[0] + Conv_Score_max_j[1])/2.0f  -  width_Inverse_Perspective/2)/(search_Lines-firsti-1 - (Conv_Score_max_i[0] + Conv_Score_max_i[1])/2.0f );

    for (int i=1;i<search_Lines - firsti;i++)
    {
        Col_Center[i+firsti] = width_Inverse_Perspective/2 + k*i;
    }
}
//
//uint8 Check_RightCircle(void)
//{
//    int full_Lines = height_Inverse_Perspective;//һ��Ҫ��������ɨ������У������ͼƬ��
//    int Conv_Core[1][3][3] = {{{1,1,1},{1,1,-1},{1,-1,-1}}};
//    //Conv_Core(1).core = [1 1 1
////                         1 1 -1
////                         1 -1 -1];
//
//
//    int Conv_Score_max[1] = {-9};
//    int Conv_Score_max_i[1] = {0};
//    int Conv_Score_max_j[1] = {0};
//    for (int i=1;i<full_Lines-1;i++)//�ӵ�һ�п�ʼ����ɨ��
//    {
//        for (int j=1;j<width_Inverse_Perspective-1;j++)
//        {
//            // ����ÿ�����ĵ�(i,j)������1����ֵ���ֱ�ȡ���ֵ����
//            int Conv_Score[1] = {0};//�洢2�������
//            int flag = 1;//����ϲ��Ϸ�
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
//    int full_Lines = height_Inverse_Perspective;//һ��Ҫ��������ɨ������У������ͼƬ��
//    int Conv_Core[1][3][3] = {{{1,1,1},{-1,1,1},{-1,-1,1}}};
//    //Conv_Core(1).core = [1 1 1
////                        -1 1 1
////                       -1 -1 1];
//
//
//    int Conv_Score_max[1] = {-9};
//    int Conv_Score_max_i[1] = {0};
//    int Conv_Score_max_j[1] = {0};
//    for (int i=1;i<full_Lines-1;i++)//�ӵ�һ�п�ʼ����ɨ��
//    {
//        for (int j=1;j<width_Inverse_Perspective-1;j++)
//        {
//            // ����ÿ�����ĵ�(i,j)������1����ֵ���ֱ�ȡ���ֵ����
//            int Conv_Score[1] = {0};//�洢2�������
//            int flag = 1;//����ϲ��Ϸ�
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
//    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
//    int start_Col[2] = {width_Inverse_Perspective/2,width_Inverse_Perspective/2};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
//    int check_lines = 2;
//    uint8 result = 0;
//    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
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
//            }//���Ҳ���ֱ���Ҳ�Ϊ255����
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
//                        }//����һ�񣬲����Ҳ���ֱ���Ҳ�Ϊ255����
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
//                        }//����һ�񣬲��������ֱ���Ҳ�Ϊ255����
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
//            }//�������ֱ�����Ϊ255����
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
//                        }//����һ�񣬲��������ֱ�����Ϊ255����
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
//                        }//����һ�񣬲����Ҳ���ֱ�����Ϊ255����
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
            uint8 water_i = second_Dot[0]+1, water_j = second_Dot[1];
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
                }
                else
                {
                    third_Dot[0] = water_i-1;
                    third_Dot[1] = water_j;
                    break;
                }
            }
            third_Dot[0] = water_i-1;
            third_Dot[1] = water_j;
        }
        else if (mode == 0)
        {
            uint8 water_i = second_Dot[0]+1, water_j = second_Dot[1];
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
                }
                else
                {
                    third_Dot[0] = water_i-1;
                    third_Dot[1] = water_j;
                    break;
                }
            }
            third_Dot[0] = water_i-1;
            third_Dot[1] = water_j;
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

        if (arccosValue < 80 && arccosValue > 0)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[(uint8)round(0.5*(third_Dot[0]+second_Dot[0]))][(uint8)round(0.5*(third_Dot[1]+second_Dot[1]))]==1)
            {
                if (third_Dot[1]-second_Dot[1]>=4)
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

        if (arccosValue < 80 && arccosValue > 0)
        {
            if (mt9v03x_image_cutted_thresholding_inversePerspective[(uint8)round(0.5*(third_Dot[0]+second_Dot[0]))][(uint8)round(0.5*(third_Dot[1]+second_Dot[1]))]==1)
            {
                if (second_Dot[1]-third_Dot[1]>=4)
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
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
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
            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
            {
                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
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
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
}


uint8 Check_Left_for_LeftCircle(void)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
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
            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
            {
                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
            {
                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
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
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
}
void Compensate_ColCenter(void)
{
    int i;
    float last_center[2]={0,0};
    for (i=0;i<search_Lines;i++) //Ѱ�Ҵ������ϵ�������׶�
    {
        if (Col_Center[i] != -2)
        {
            last_center[0] = Col_Center[i];//j
            last_center[1] = (float)i;//i
            break;
        }
    }

    //�洢�ײ��е�����

    float buttom_center[2]={0,0};
    for (i=0;i<search_Lines;i++) //Ѱ����Ұ�ײ�
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
        //��ȫ�յ�����ߺ͵ײ�����֮�������
        float k = (last_center[0]  - buttom_center[0])/(last_center[1]  - buttom_center[1]);

        for (int i=0;i<(last_center[1]-buttom_center[1]);i++)
        {
            Col_Center[i+firsti] = buttom_center[0] + k*i;
        }
    }


}


uint8 Check_Right_Straight_forRightCircle(int8 max_d_Col_Right, int8 min_d_Col_Right, float ratio)
{
    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
    for (int i=0;i<search_Lines;i++)
    {
        Col_Left[i] = -2;
        Col_Right[i] = -2;
    }
    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
    {
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 255)
        {
            start_Row = start_Row - 1;
            continue;
        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 0)//������߷���0���򣨱�����
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
            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]] == 1)
            {
                Col_Right[i] = start_Col[1];//ֻ����1����Ĳſ��Խ��кŴ洢��������
            }
        }
        else//������߷���1���򣨵�·��
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
            start_Col[1] = start_Col[1] - 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[1]+1] == 0)
            {
                Col_Right[i] = start_Col[1];//ֻ����0����Ĳſ��Խ��кŴ洢��������
            }
        }

        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
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
        }
        start_Row = start_Row - 1;
    }
    if (d_Col_Right_Num<=0.7*ratio*Search_Range[ROW][LINES])
    {
        return 0;
    }
    for (int j=0;j<d_Col_Right_Num;j++)
    {
        if (Col_Right[i+1]==-2 || Col_Right[i]==-2)
        {
            i++;
            continue;
        }
        d_Col_Right = Col_Right[i+1]-Col_Right[i];
        i++;
        if (d_Col_Right < min_d_Col_Right || d_Col_Right > max_d_Col_Right)
        {
            return 0;
        }
    }
    return 1;
}


//uint8 Check_Left_Straight_forLeftCircle(int8 max_d_Col_Left, int8 min_d_Col_Left, float ratio)
//{
//    int start_Row = height_Inverse_Perspective-1;//��ǵ�ǰ�ڴ�����һ�У������һ�п�ʼ
//    int start_Col[2] = {width_Inverse_Perspective/2-5,width_Inverse_Perspective/2+5};//��ǵ�ǰ�ڴ�����һ�У�start_Col(1)ָ���ߣ�start_Col(2)ָ���ߣ�Ĭ�ϴ���������5���ؿ�ʼ
//    for (int i=0;i<search_Lines;i++)
//    {
//        Col_Left[i] = -2;
//        Col_Right[i] = -2;
//    }
//    for (int i=0;i<search_Lines;i++)//�����һ�п�ʼ����ɨ�裬һ��ɨ��search_Lines��
//    {
//        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 255)
//        {
//            start_Row = start_Row - 1;
//            continue;
//        }//�ڵ�ǰ�����������߻�������һ����255����δ֪���򣩣�˵����û�н��뵽�������ӽ�����0�����1���򣩣�������������ܵ����ߵ��ұ�ȥ�ˣ���˵�����ǵ�·��
//        if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1) //������߷���1���򣨵�·��
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1 && start_Col[0]>=1)
//            {
//                start_Col[0] = start_Col[0] - 1;
//                cnt_temp = cnt_temp + 1;
//            }
//            if (cnt_temp>2*road_width)
//            {
//                break;
//            }
//            start_Col[0] = start_Col[0] + 1;//�����߳�������ɨ��ֱ��������1���򣨵�·�����п�����0��255����
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]-1] == 0)//�鿴��ʱ�Ƿ���0���򣨱�����
//            {
//                Col_Left[i] = start_Col[0];//ֻ����0����Ĳſ��Խ��кŴ洢��������
//            }
//        }
//        else//������߷���0���򣨱�����
//        {
//            int cnt_temp = 0;
//            while (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 0 && start_Col[0]<=width_Inverse_Perspective-2)
//            {
//                start_Col[0] = start_Col[0] + 1;
//                cnt_temp = cnt_temp + 1;
//            }
//            if (cnt_temp>2*road_width)
//            {
//                break;
//            }//�����߳�������ɨ��ֱ��������0���򣨱��������п�����1��255����
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][start_Col[0]] == 1)//�鿴��ʱ�Ƿ���1���򣨵�·��
//            {
//                Col_Left[i] = start_Col[0];//ֻ����1����Ĳſ��Խ��кŴ洢��������
//            }
//        }
//
//        start_Row = start_Row - 1; //������ɨ����ϣ�����н�����һ�У�����һ��������ɨ����׼��
//    }
//
//    start_Row = Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]-1;
//    int d_Col_Left;
//    int d_Col_Left_Num=0;
//
//    int i;
//    for (i=height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+Search_Range[ROW][LINES]);i<height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES]);i++)
//    {
//        if (Col_Left[i] !=-2)
//        {
//            if (mt9v03x_image_cutted_thresholding_inversePerspective[start_Row][Col_Left[i]-1] == 0)
//            {
//                d_Col_Left_Num = height_Inverse_Perspective - (Search_Range[ROW][BEGIN]+(1-ratio)*Search_Range[ROW][LINES])-i-1;
//                break;
//            }
//        }
//        start_Row = start_Row - 1;
//    }
//    if (d_Col_Left_Num<=0.7*ratio*Search_Range[ROW][LINES])
//    {
//        return 0;
//    }
//    for (int j=0;j<d_Col_Left_Num;j++)
//    {
//        if (Col_Left[i+1]==-2 || Col_Left[i]==-2)
//        {
//            i++;
//            continue;
//        }
//        d_Col_Left = Col_Left[i+1]-Col_Left[i];
//        i++;
//        if (d_Col_Left < min_d_Col_Left || d_Col_Left > max_d_Col_Left)
//        {
//            return 0;
//        }
//    }
//    return 1;
//}
