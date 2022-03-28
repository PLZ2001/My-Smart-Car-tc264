#include "headfile.h"
#include "WIFI.h"
#include "CAMERA.h"

float variable_Group[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8 image_Edge[300];//ע�ⲻҪ���

void My_Init_Wifi(void)
{
    uart_init(DEBUG_UART, 1500000, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
}

void Send_with_Wifi(void)
{
    Send_Begin();
    Send_Variable();
    Send_Image(*mt9v03x_image_cutted_thresholding_inversePerspective, width_Inverse_Perspective, height_Inverse_Perspective, 80, 60);
}

void Send_Begin(void)
{
    uart_putchar(DEBUG_UART, 0x55);//85
    uart_putchar(DEBUG_UART, 0xAA);//170
    uart_putchar(DEBUG_UART, 0x11);//17
}

void Send_Variable(void)
{
    uart_putchar(DEBUG_UART, 0x55);//85
    uart_putchar(DEBUG_UART, 0xAA);//170
    uart_putchar(DEBUG_UART, 0xFF);//255
    uart_putchar(DEBUG_UART, 0x01);//1

    uint8 variable_Num = 16;
    uart_putchar(DEBUG_UART, variable_Num);

    for(uint8 i=0;i<variable_Num;i++)
    {
        float temp = variable_Group[i];
        uart_putchar(DEBUG_UART, BYTE0(temp));
        uart_putchar(DEBUG_UART, BYTE1(temp));
        uart_putchar(DEBUG_UART, BYTE2(temp));
        uart_putchar(DEBUG_UART, BYTE3(temp));
    }

    uart_putchar(DEBUG_UART, 0x01);//1
}

void Send_Image(uint8* image_Address, uint8 image_Width, uint8 image_Height, uint8 processed_Width, uint8 processed_Height)
{
    uart_putchar(DEBUG_UART, 0x55);//85
    uart_putchar(DEBUG_UART, 0xAA);//170
    uart_putchar(DEBUG_UART, 0xFF);//255
    uart_putchar(DEBUG_UART, 0xA2);//162

    uart_putchar(DEBUG_UART, 0x00);//0

    uint16 cont = Get_Edge(image_Address, image_Width, image_Height, processed_Width, processed_Height);
    uint16 bytes_Send = cont+2;
    uart_putchar(DEBUG_UART, BYTE0(bytes_Send));
    uart_putchar(DEBUG_UART, BYTE1(bytes_Send));

    for (int16 i=0;i<cont;i++)
    {
        uart_putchar(DEBUG_UART, image_Edge[i]);
    }

    uart_putchar(DEBUG_UART, 0xF0);//240
    uart_putchar(DEBUG_UART, 0xF2);//242

}

// Get_Edge��Ŀ���ǽ�һ�������Ķ�ֵ��ͼ��ת����һ���ر��һά����image_Edge�����洢ÿһ�г��ֺڰ�������кţ�����255��Ϊÿһ�н����ı�־
// Get_Edge������Чѹ��ͼƬ�ֽ���
uint16 Get_Edge(uint8* image_Address, uint8 image_Width, uint8 image_Height, uint8 processed_Width, uint8 processed_Height)
{
    uint8 last=0;
    uint8 temp=0;
    uint16 cont=0;

    //��Ҫ��ͼƬѹ����processed_Widthxprocessed_Height��Χ��
    float width_Scale = 1.0f*image_Width/processed_Width;
    float height_Scale = 1.0f*image_Height/processed_Height;

    for(uint8 i=0;i<processed_Height;i++)
    {
        last=0;
        for(uint8 j=0;j<processed_Width;j++)
        {

            temp = image_Address[(uint16)(round(i * height_Scale) * image_Width + round(j * width_Scale))];// ��ȡ�õ�����ֵ ��0��1��
            if (temp==1||temp==0)
            {
                if (temp!=last) //����һ��ֵ����ͬ ������������
                {
                    image_Edge[cont++] = j;
                }
                last = temp;  //�洢�õ��ֵ
            }

        }
        image_Edge[cont++] = 0xFF;
    }
    return cont;
}
