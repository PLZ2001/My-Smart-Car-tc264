#include "headfile.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"
#include "OLED.h"
#include "MOTOR_CTL.h"
#include "TIME.h"
//#include "EEPROM.h"

float steering_Error = 0;//��ǰͼ���µ�ʵ�����������������ߵ����
float d_steering_Error = 0;
uint32 STEERING_DUTY_CENTER=748;//774;//762;//756;//772;//766;//760;//774;//756;//754;//769;//759;//771;//779;//774;//770;//;773;//765;//779;//781;//755;//775;//765;//777;//671;//667;//661;//669;//643;//652;//646;//667;//639;//653;//644;//646;//664;//652;//665;//647;//1500;//1772;

float SightForward = 0.25f;//0.54f;
float SightForward_Highest = 0.25f;
float SightForward_High = 0.25f;
float SightForward_Low = 0.25f;
float SightForward_Lowest = 0.25f;
float SightForward_Lowest_ForT = 0.25f;

float kp,kd;

uint8 TurnTime = 10;

struct steerpid
{
    float last_error;//�ϴ�
    float current_error; //��ǰ
    float KP;
    float KI;
    float KD;
} Steering_PID={0.0f,0.0f,0.25f,0.0f,0.3f};


//��Ҫ����ͨ�Ŵ������ı�������������ִ�б������µĺ�����
float steering_Target = 0;//Ŀ��Ƕȣ��㣩�����º���Set_Steering_Target(uint8 val)����Χ-30��30��



void My_Init_Steering(void)
{
    gtm_pwm_init(ATOM2_CH0_P33_10, 50, STEERING_DUTY_CENTER);//����P33.10���PWM����Ƶ��100Hz��ռ�ձ�1520/10000�����ڶ��
}

void UART_Steering(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    uart_putchar(DEBUG_UART, (uint8)round((steering_Target - STEERING_MIN)/(STEERING_MAX-STEERING_MIN)*255));
    int16 se  = (int16)round(100*Steering_PID.current_error);
    uart_putchar(DEBUG_UART, se>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, se&0x00FF);//�ȴ���8λ���ٴ���8λ
    int16 d_se  = (int16)round(100*(Steering_PID.current_error-Steering_PID.last_error));
    uart_putchar(DEBUG_UART, d_se>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, d_se&0x00FF);//�ȴ���8λ���ٴ���8λ
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
    int cnt_start = 0;
    float di = Cal_Steering_Range_of_Img/0.5f;
    float last_Col_Center=-2;
    for (float temp_i=0;temp_i<(search_Lines*Cal_Steering_Range_of_Img);temp_i+=di)
    {
        int i = floor(temp_i+0.5);
        if(Col_Center[i] != -2)
        {
            cnt = cnt+1;
            if (cnt == 1)
            {
                cnt_start = i;
            }
            float ratio = (i-cnt_start)*1.0f/(cnt_start-search_Lines*Cal_Steering_Range_of_Img)*(1.1f-0.9f)+1.1f;
            steering_Error_tmp = steering_Error_tmp + (Col_Center[i] - Col_Center_Init)*ratio;
            last_Col_Center = Col_Center[i];
        }
        else if (Col_Center[i] == -2 && cnt>=1)
        {
            float ratio = (i-cnt_start)*1.0f/(cnt_start-search_Lines*Cal_Steering_Range_of_Img)*(1.1f-0.9f)+1.1f;
            steering_Error_tmp = steering_Error_tmp + (last_Col_Center - Col_Center_Init)*ratio;
        }
    }

    steering_Error = steering_Error_tmp;

}

//https://blog.q_49487109/article/details/117963017csdn.net/q �ɲο�
void Cal_Steering_Target(void)
{
    //����ȫ�ֱ����������壩����λ��ʽPDԭ����ת��Ŀ��Steering_Target(��Χ-30~30��������ת��������ת)
    Steering_PID.last_error = Steering_PID.current_error;
    Steering_PID.current_error = steering_Error;
    d_steering_Error = Steering_PID.current_error-Steering_PID.last_error;

    float K_kp=0.4f,K_kd=0.4f;
    kp = Steering_PID.KP + (steering_Error/1000)*(steering_Error/1000)*K_kp;
//    kd = Steering_PID.KD + (d_steering_Error/100)*(d_steering_Error/100)*K_kd;
    kd = Steering_PID.KD - (steering_Error/1000)*(steering_Error/1000)*K_kd;
    //"0����", "1����", "2�󻷵�", "3�һ���", "4����·��", "5ʮ��·��","6ֱ��","7������ʱʹ�ã�","8���ң���ʱʹ�ã�", "9δ֪"

    switch(classification_Result){
//        case 5:
//        {
//             steering_Target = (kp/1.3f * Steering_PID.current_error) +kd/1.3f*( Steering_PID.current_error - Steering_PID.last_error );
//             break;
//        }
        case 6:
        {
            steering_Target = (kp/1.3f * Steering_PID.current_error) +kd/1.3f*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }
        case 7:
        {
            if (flag_For_T == 2)
            {
                steering_Target = STEERING_MIN-10;
            }
            else
            {
                steering_Target = (kp * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
            }
            break;
        }
        case 8:
        {
            if (flag_For_T == 2)
            {
                steering_Target = STEERING_MAX+10;
            }
            else
            {
                steering_Target = (kp * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
            }
            break;
        }
        case 2:
        {
//            if (flag_For_Left_Circle==1)
//            {
//                steering_Target = -35.0f;
//            }
//            else
//            {
                steering_Target = (kp * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
//            }
            break;
        }
        case 3:
        {
//            if (flag_For_Right_Circle==1)
//            {
//                steering_Target = 35.0f;
//            }
//            else
//            {
                steering_Target = (kp * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
//            }
            break;
        }
        case 10:
        case 11:
        {
            steering_Target = (kp/1.3f * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }
//        case 0:
//        case 1:
//        case 9:
//        {
//            steering_Target = (Steering_PID.KP*1.3f * Steering_PID.current_error) +Steering_PID.KD*1.3f*( Steering_PID.current_error - Steering_PID.last_error );
//            break;
//        }

        default:
        {
            steering_Target = (kp * Steering_PID.current_error) +kd*( Steering_PID.current_error - Steering_PID.last_error );
            break;
        }
    }

    if (zebra_status==starting)
    {
        static float steering_Target_Remember = 0;
        static uint8 steering_Target_Remember_flag=0;
        if (steering_Target_Remember_flag==0)
        {
            steering_Target=0;
            if (Check_TRoad(1,0.12f,7) == 1)
            {
                if(zebra_start_direction==1)
                {
                    steering_Target = STEERING_MAX+10;
                    steering_Target_Remember = steering_Target;
                    steering_Target_Remember_flag=1;
                }
                else if(zebra_start_direction==-1)
                {
                    steering_Target = STEERING_MIN-10;
                    steering_Target_Remember = steering_Target;
                    steering_Target_Remember_flag=1;
                }
            }
        }
        else
        {
            steering_Target = steering_Target_Remember;
        }

        if (steering_Target == steering_Target_Remember  && (Left_Straight_Score>=2.55f||Unknown_Straight_Score>=2.55f||Right_Straight_Score>=2.55f))
        {
            zebra_status=finding;
        }
    }
    if (zebra_status==finishing)
    {
        static float steering_Target_Remember = 0;
        static uint8 steering_Target_Remember_flag=0;
        if (steering_Target_Remember_flag==0)
        {
            if (zebra_direction==-1)
            {
                steering_Target = STEERING_MIN-10;
                steering_Target_Remember = steering_Target;
                steering_Target_Remember_flag=1;
            }
            else if(zebra_direction==1)
            {
                steering_Target = STEERING_MAX+10;
                steering_Target_Remember = steering_Target;
                steering_Target_Remember_flag=1;
            }
        }
        else
        {
            steering_Target = steering_Target_Remember;
        }

        if (Left_Straight_Score<=1.50f&&Unknown_Straight_Score<=1.50f&&Right_Straight_Score<=1.50f)
        {
            if (Left_Straight_Score>=1.0f&&Unknown_Straight_Score>=1.0f&&Right_Straight_Score>=1.0f)
            {
                emergency_Stop=1;
            }
        }
    }

    //if(steering_Target<0) steering_Target = steering_Target*1.1;
    if(steering_Target>STEERING_MAX+10) steering_Target = STEERING_MAX+10;
    if(steering_Target<STEERING_MIN-10) steering_Target = STEERING_MIN-10;
}


void Key_temp(void)
{
    switch (pointer_temp)
    {
        case 0:
            Steering_PID.KP += 0.05f*up_Down;
            break;
        case 1:
            Steering_PID.KI += 0.05f*up_Down;
            break;
        case 2:
            Steering_PID.KD += 0.05f*up_Down;
            break;
        default:
            break;
    }
}

void OLED_temp(void)
{
    if (pointer_temp == 0)
    {
        OLED_PRINTF(0,0,"->Kp:%01.02f s   ",Steering_PID.KP);
        OLED_PRINTF(0,1,"Ki:%01.02f s   ",Steering_PID.KI);
        OLED_PRINTF(0,2,"Kd:%01.02f s   ",Steering_PID.KD);
    }
    else if (pointer_temp == 1)
    {
        OLED_PRINTF(0,0,"Kp:%01.02f s   ",Steering_PID.KP);
        OLED_PRINTF(0,1,"->Ki:%01.02f s   ",Steering_PID.KI);
        OLED_PRINTF(0,2,"Kd:%01.02f s   ",Steering_PID.KD);
    }
    else if (pointer_temp == 2)
    {
        OLED_PRINTF(0,0,"Kp:%01.02f s   ",Steering_PID.KP);
        OLED_PRINTF(0,1,"Ki:%01.02f s   ",Steering_PID.KI);
        OLED_PRINTF(0,2,"->Kd:%01.02f s   ",Steering_PID.KD);
    }
    if (up_Down == 1)
    {
        OLED_PRINTF(0,3,"ADD");
    }
    else if (up_Down == -1)
    {
        OLED_PRINTF(0,3,"SUB");
    }
}

void Change_Steering_PID(float kp, float ki, float kd)
{
    Steering_PID.KP = kp;
    Steering_PID.KI = ki;
    Steering_PID.KD = kd;
}
