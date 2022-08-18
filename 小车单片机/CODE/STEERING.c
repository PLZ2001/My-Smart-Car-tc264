#include "headfile.h"
#include "STEERING.h"
#include "CAMERA.h"
#include "SEARCH.h"
#include "OLED.h"
#include "MOTOR_CTL.h"
#include "TIME.h"
//#include "EEPROM.h"

float steering_Error = 0;//当前图像下的实际中线与理想正中线的误差
float d_steering_Error = 0;
uint32 STEERING_DUTY_CENTER=772;//769;//767;//765;//773;//747;//763;//777;//755;//771;//763;//773;//751;//775;//757;//773;//767;//771;//749;//767;//747;//777;//763;//777;//771;//781;//769;//739;//765;//781;//757;//767;//751;//767;//751;//763;//777;//745;//751;//781;//763;//777;//743;//779;//773;//779;//765;//777;//771;//761;//747;//777;//749;//769;//753;//775;//739;//771;//743;//773;//767;//773;//755;//777;//763;//775;//773;//767;//773;//757;//773;//767;//757;//773;//757;//773;//763;//772;//754;//772;//764;//776;//764;//776;//768;//760;//774;//762;//772;//756;//772;//762;//774;//768;//748;//760;//750;//768;//758;//772;//764;//754;//772;//758;//774;//762;//772;//764;//752;//766;//754;//770;//756;//772;//766;//756;//772;//760;//772;//778;//768;//774;//748;//774;//762;//756;//772;//766;//760;//774;//756;//754;//769;//759;//771;//779;//774;//770;//;773;//765;//779;//781;//755;//775;//765;//777;//671;//667;//661;//669;//643;//652;//646;//667;//639;//653;//644;//646;//664;//652;//665;//647;//1500;//1772;

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
    float last_error;//上次
    float current_error; //当前
    float KP;
    float KI;
    float KD;
} Steering_PID={0.0f,0.0f,0.25f,0.0f,0.3f};


//需要串口通信传过来的变量（必须配以执行变量更新的函数）
float steering_Target = 0;//目标角度（°），更新函数Set_Steering_Target(uint8 val)（范围-30，30）



void My_Init_Steering(void)
{
    gtm_pwm_init(ATOM2_CH0_P33_10, 50, STEERING_DUTY_CENTER);//设置P33.10输出PWM波，频率100Hz，占空比1520/10000；用于舵机
}

void UART_Steering(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x05);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, (uint8)round((steering_Target - STEERING_MIN)/(STEERING_MAX-STEERING_MIN)*255));
    int16 se  = (int16)round(100*Steering_PID.current_error);
    uart_putchar(DEBUG_UART, se>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, se&0x00FF);//先传高8位，再传低8位
    int16 d_se  = (int16)round(100*(Steering_PID.current_error-Steering_PID.last_error));
    uart_putchar(DEBUG_UART, d_se>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, d_se&0x00FF);//先传高8位，再传低8位
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
    float Col_Center_Init = width_Inverse_Perspective/2.0f;


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
            float ratio = (i-cnt_start)*1.0f/(cnt_start-search_Lines*Cal_Steering_Range_of_Img)*(1.3f-0.7f)+1.3f;//改
            steering_Error_tmp = steering_Error_tmp + (Col_Center[i] - Col_Center_Init)*ratio;
            last_Col_Center = Col_Center[i];
        }
        else if (Col_Center[i] == -2 && cnt>=1)
        {
            float ratio = (i-cnt_start)*1.0f/(cnt_start-search_Lines*Cal_Steering_Range_of_Img)*(1.3f-0.7f)+1.3f;
            steering_Error_tmp = steering_Error_tmp + (last_Col_Center - Col_Center_Init)*ratio;
        }
    }

    static float last_steering_Error = 0;
    static float last_sight_forward = 0;
    static int cnt_temp=0;
    if (((steering_Error_tmp>150.0f)&&(steering_Error_tmp - last_steering_Error<-30.0f))
            ||((steering_Error_tmp<-150.0f)&&(steering_Error_tmp - last_steering_Error>30.0f))
            ||(steering_Error_tmp - last_steering_Error>60.0f)
            ||(steering_Error_tmp - last_steering_Error<-60.0f))
    {
        cnt_temp++;
        if (cnt_temp>=2||(last_sight_forward!=SightForward))
        {
            cnt_temp=0;
            steering_Error = steering_Error_tmp;
            last_steering_Error = steering_Error;
        }
    }
    else
    {
        cnt_temp=0;
        steering_Error = steering_Error_tmp;
        last_steering_Error = steering_Error;
    }
    last_sight_forward = SightForward;

}

//https://blog.q_49487109/article/details/117963017csdn.net/q 可参考
void Cal_Steering_Target(void)
{
    //由误差（全局变量，待定义）根据位置式PD原理求转向目标Steering_Target(范围-30~30，负数左转，正数右转)
    Steering_PID.last_error = Steering_PID.current_error;
    Steering_PID.current_error = steering_Error;
    d_steering_Error = Steering_PID.current_error-Steering_PID.last_error;



    float K_kp=2.8f,K_kd=3.0f;
    if (classification_Result==2||classification_Result==3||zebra_status == finishing)
    {
        kp = Steering_PID.KP;
        kd = Steering_PID.KD;
    }
    else
    {
        kp = Steering_PID.KP + (steering_Error/1000)*(steering_Error/1000)*K_kp;
        if (kp>0.16)
        {
            kp=0.16;
        }
//        kd = Steering_PID.KD + (d_steering_Error/100)*(d_steering_Error/100)*K_kd- (steering_Error/1000)*(steering_Error/1000)*K_kd;
        kd = Steering_PID.KD - (steering_Error/1000)*(steering_Error/1000)*K_kd;
        if(kd<0)
        {
            kd=0;
        }
    }

    //"0左弯", "1右弯", "2左环岛", "3右环岛", "4三岔路口", "5十字路口","6直道","7靠左（临时使用）","8靠右（临时使用）", "9未知"

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
            Set_Search_Range(height_Inverse_Perspective*4/10,height_Inverse_Perspective-height_Inverse_Perspective*4/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
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
            Set_Search_Range(height_Inverse_Perspective*3/10,height_Inverse_Perspective*9/10-height_Inverse_Perspective*3/10,width_Inverse_Perspective/4,width_Inverse_Perspective-width_Inverse_Perspective/4*2);
        }
        else
        {
            steering_Target = steering_Target_Remember;
        }
        static uint8 ready_run = 0;
        if ((steering_Target == STEERING_MAX+10 || steering_Target == STEERING_MIN-10)  && (Left_Straight_Score<=1.7f&&Unknown_Straight_Score<=1.7f&&Right_Straight_Score<=1.7f))
        {
            ready_run = 1;
        }
        if ((steering_Target == STEERING_MAX+10 || steering_Target == STEERING_MIN-10)  && ready_run == 1&&(Left_Straight_Score>=2.0f||Unknown_Straight_Score>=2.0f||Right_Straight_Score>=2.0f))
        {
            zebra_status=finding;
        }
    }
    if (zebra_status==finishing)
    {
//        static float steering_Target_Remember = 0;
//        static uint8 steering_Target_Remember_flag=0;
//        if (steering_Target_Remember_flag==0)
//        {
//            if (zebra_direction==-1)
//            {
//                steering_Target = STEERING_MIN-10;
//                steering_Target_Remember = steering_Target;
//                steering_Target_Remember_flag=1;
//            }
//            else if(zebra_direction==1)
//            {
//                steering_Target = STEERING_MAX+10;
//                steering_Target_Remember = steering_Target;
//                steering_Target_Remember_flag=1;
//            }
//        }
//        else
//        {
//            steering_Target = steering_Target_Remember;
//        }
//
        if (Left_Straight_Score<=1.50f&&Unknown_Straight_Score<=1.50f&&Right_Straight_Score<=1.50f)
        {
            if (Left_Straight_Score>=1.0f&&Unknown_Straight_Score>=1.0f&&Right_Straight_Score>=1.0f)
            {
                Stop=0;
            }
        }
    }

    //if(steering_Target<0) steering_Target = steering_Target*1.1;
//    if(steering_Target>0)
//    {
//        steering_Target = 0.86*steering_Target;
//    }
//    else if(steering_Target<0)
//    {
//        steering_Target = 1.23*steering_Target;
//    }
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
