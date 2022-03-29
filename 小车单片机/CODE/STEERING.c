#include "headfile.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"

float steering_Error = 0;//��ǰͼ���µ�ʵ�����������������ߵ����

struct
{
    float last_error;//�ϴ�
    float current_error; //��ǰ
    float KP;
    float KI;
    float KD;
}Steering_PID={0.0f,0.0f,0.2f,0.0f,0.0f};


//��Ҫ����ͨ�Ŵ������ı�������������ִ�б������µĺ�����
float steering_Target = 0;//Ŀ��Ƕȣ��㣩�����º���Set_Steering_Target(uint8 val)����Χ-30��30��



void My_Init_Steering(void)
{
    gtm_pwm_init(ATOM2_CH0_P33_10, 100, STEERING_DUTY_CENTER);//����P33.10���PWM����Ƶ��100Hz��ռ�ձ�1520/10000�����ڶ��
}

void UART_Steering(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    uart_putchar(DEBUG_UART, (uint8)round((steering_Target - STEERING_MIN)/(STEERING_MAX-STEERING_MIN)*255));
    int16 se  = (int16)round(100*steering_Error);
    uart_putchar(DEBUG_UART, se>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, se&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void Set_Steering_Target(uint8 val)
{
    steering_Target = ((float)val) / 256 * (STEERING_MAX-STEERING_MIN)+STEERING_MIN;
}

void UART_SteeringPID(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x06);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    int16 kp = (int16)round(1000*Steering_PID.KP);
    int16 ki = (int16)round(1000*Steering_PID.KI);
    int16 kd = (int16)round(1000*Steering_PID.KD);
    uart_putchar(DEBUG_UART, kp>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kp&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, ki>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, ki&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kd>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kd&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x06);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void Set_SteeringPID(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6)
{
    Steering_PID.KP = (v1*256.0f+v2)/1000.0f;
    Steering_PID.KI = (v3*256.0f+v4)/1000.0f;
    Steering_PID.KD = (v5*256.0f+v6)/1000.0f;
}

void Set_Steering(void)
{
    uint32 duty;
    duty = STEERING_DUTY_MAX/STEERING_MAX*(steering_Target>0?steering_Target:(-steering_Target));
    if (duty>STEERING_DUTY_MAX)//�������
    {
        duty = STEERING_DUTY_MAX;
    }
    if (steering_Target == 0)
    {
        duty = 0;
    }
    pwm_duty(ATOM2_CH0_P33_10, (uint32)(STEERING_DUTY_CENTER+(steering_Target<0?1:-1)*duty));
}

void Cal_Steering_Error(float Cal_Steering_Range_of_Img)
{
    //����Col_Center��ɨ�跶Χsearch_Lines������ȫ�ֱ�����

    float steering_Error_tmp = 0;
    float Col_Center_Init = width_Inverse_Perspective/2;


    int cnt = 0;
    for (int i=0;i<(search_Lines*Cal_Steering_Range_of_Img);i++)
    {
        if(Col_Center[i] != -2)
        {
            cnt = cnt+1;
            steering_Error_tmp = steering_Error_tmp + Col_Center[i] - Col_Center_Init;
        }
    }

    steering_Error = steering_Error_tmp*(113.0f*59.0f)/(width_Inverse_Perspective*1.0f*height_Inverse_Perspective);

}

//https://blog.q_49487109/article/details/117963017csdn.net/q �ɲο�
void Cal_Steering_Target(void)
{
    //����ȫ�ֱ����������壩����λ��ʽPDԭ����ת��Ŀ��Steering_Target(��Χ-30~30��������ת��������ת)
    Steering_PID.last_error = Steering_PID.current_error;
    Steering_PID.current_error = steering_Error;

    //"0����", "1����", "2�󻷵�", "3�һ���", "4����·��", "5ʮ��·��","6ֱ��","7������ʱʹ�ã�","8���ң���ʱʹ�ã�", "9δ֪"

    switch(classification_Result){
        case 5:
        {
             steering_Target = (Steering_PID.KP/10 * Steering_PID.current_error) +Steering_PID.KD/10*( Steering_PID.current_error - Steering_PID.last_error );
             break;
        }
        case 6:
        {
            steering_Target = (Steering_PID.KP/10 * Steering_PID.current_error) +Steering_PID.KD/10*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }
        case 8:
        {
            steering_Target = (Steering_PID.KP/1.3f * Steering_PID.current_error) +Steering_PID.KD/1.3f*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }

        default:
        {
            steering_Target = (Steering_PID.KP * Steering_PID.current_error) +Steering_PID.KD*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }
    }



    //if(steering_Target<0) steering_Target = steering_Target*1.1;
    if(steering_Target>STEERING_MAX) steering_Target = STEERING_MAX;
    if(steering_Target<STEERING_MIN) steering_Target = STEERING_MIN;
}
