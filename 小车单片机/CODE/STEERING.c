#include "headfile.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"

float steering_Error = 0;//当前图像下的实际中线与理想正中线的误差

struct
{
    float last_error;//上次
    float current_error; //当前
    float KP;
    float KI;
    float KD;
}Steering_PID={0.0f,0.0f,0.2f,0.0f,0.0f};


//需要串口通信传过来的变量（必须配以执行变量更新的函数）
float steering_Target = 0;//目标角度（°），更新函数Set_Steering_Target(uint8 val)（范围-30，30）



void My_Init_Steering(void)
{
    gtm_pwm_init(ATOM2_CH0_P33_10, 100, STEERING_DUTY_CENTER);//设置P33.10输出PWM波，频率100Hz，占空比1520/10000；用于舵机
}

void UART_Steering(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, (uint8)round((steering_Target - STEERING_MIN)/(STEERING_MAX-STEERING_MIN)*255));
    int16 se  = (int16)round(100*steering_Error);
    uart_putchar(DEBUG_UART, se>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, se&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
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
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    int16 kp = (int16)round(1000*Steering_PID.KP);
    int16 ki = (int16)round(1000*Steering_PID.KI);
    int16 kd = (int16)round(1000*Steering_PID.KD);
    uart_putchar(DEBUG_UART, kp>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kp&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, ki>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, ki&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kd>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kd&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x06);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
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
    if (duty>STEERING_DUTY_MAX)//保护电机
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
    //根据Col_Center和扫描范围search_Lines计算误差（全局变量）

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

//https://blog.q_49487109/article/details/117963017csdn.net/q 可参考
void Cal_Steering_Target(void)
{
    //由误差（全局变量，待定义）根据位置式PD原理求转向目标Steering_Target(范围-30~30，负数左转，正数右转)
    Steering_PID.last_error = Steering_PID.current_error;
    Steering_PID.current_error = steering_Error;

    //"0左弯", "1右弯", "2左环岛", "3右环岛", "4三岔路口", "5十字路口","6直道","7靠左（临时使用）","8靠右（临时使用）", "9未知"

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
