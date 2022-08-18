#include "headfile.h"
#include "IfxGpt12_reg.h"
#include "IfxGpt12.h"
#include "fuzzy_PID.h"//模糊PID算法
#include "MOTOR1.h"
#include "MOTOR_CTL.h"


//1代表右电机


//需要串口通信输出去，但不用传过来的变量
float speed_Measured1 = 0;//测量速度（m/s）


//需要串口通信传过来的变量（必须配以执行变量更新的函数）
float speed_Target1 = 0;//目标速度（m/s），更新函数Set_Speed_Target(uint8 val)


//其他变量
float speed_Output1 = 0;//输出速度（m/s）
float PID_KP1=7.0f/1.0f;//7.0f/3.0f;//7.0f/0.5f;//7.0f/3.0f;
float PID_KI1=0.5f;//0.15f;//0.04f;//0.01f;
float PID_KD1=0.0f;//0.0f;

enum PID_Mode1 PID_mode1 = PID_CLOSED_LOOP1;//PID模式选择

float BANGBANG_UP1=0.2;
float BANGBANG_DOWN1=0.3;

void My_Init_Motor1(void)
{
    gtm_pwm_init(ATOM0_CH6_P23_1, 12500, 0);//设置P23.1输出PWM波，频率12.5kHz，占空比5000/10000；用于直流电机正端
    gtm_pwm_init(ATOM1_CH5_P32_4, 12500, 0);//设置P32.4输出PWM波，频率12.5kHz，占空比5000/10000；用于直流电机负端
}

void My_Init_SpeedSensor1(void)
{
    gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);
    pit_interrupt_ms(CCU6_1, PIT_CH0, SPEED_MEASURING_PERIOD_ms1);
}

void UART_Speed1(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x04);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    uart_putchar(DEBUG_UART, (uint8)round((speed_Measured1 - SPEED_MIN1)/(SPEED_MAX1-SPEED_MIN1)*255));
    uart_putchar(DEBUG_UART, (uint8)round((speed_Output1 - SPEED_MIN1)/(SPEED_MAX1-SPEED_MIN1)*255));
    uart_putchar(DEBUG_UART, (uint8)round((speed_Target1 - SPEED_MIN1)/(SPEED_MAX1-SPEED_MIN1)*255));
    uart_putchar(DEBUG_UART, (uint8)round((pid_vector[0]->last_error - SPEED_MIN1)/(SPEED_MAX1-SPEED_MIN1)*255));
    uart_putchar(DEBUG_UART, (uint8)round((pid_vector[0]->current_error - SPEED_MIN1)/(SPEED_MAX1-SPEED_MIN1)*255));
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x04);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void Set_Speed_Target1(uint8 val)
{
    speed_Target1 = ((float)val) / 256 * (SPEED_MAX1-SPEED_MIN1)+SPEED_MIN1;//因为速度范围是对称的，而255不对称，所以接收时用256
}


void UART_PID1(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x07);
    uart_putchar(DEBUG_UART,0x01);//发送数据头
    int16 kp = (int16)round(1000*PID_KP1);
    int16 ki = (int16)round(1000*PID_KI1);
    int16 kd = (int16)round(1000*PID_KD1);
    uart_putchar(DEBUG_UART, kp>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kp&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, ki>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, ki&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kd>>8);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART, kd&0x00FF);//先传高8位，再传低8位
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x07);
    uart_putchar(DEBUG_UART,0x02);//发送数据尾
}

void Set_PID1(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6)
{
    PID_KP1 = (v1*256.0f+v2)/1000.0f;
    PID_KI1 = (v3*256.0f+v4)/1000.0f;
    PID_KD1 = (v5*256.0f+v6)/1000.0f;
}

void Get_Speed_perSPEED_MEASURING_PERIOD_ms1(void)
{
    uint16 count = gpt12_get(GPT12_T2);
    gpt12_clear(GPT12_T2);
    if (count <= 32767)
    {
        speed_Measured1 = ((float)count)*0.033*2*3.1415926/(1.0*SPEED_MEASURING_PERIOD_ms1/1000.0)/1024.0;
    }
    else
    {
        speed_Measured1 = ((float)count-65536)*0.033*2*3.1415926/(1.0*SPEED_MEASURING_PERIOD_ms1/1000.0)/1024.0;
    }
    // (编码器计数/计数周期)/1024 = 转速（转/s），(编码器计数/计数周期)/1024*0.033*2*3.1415926 = 速度（m/s）
}


/*通过T5的Capture模式来测速
void My_Init_SpeedSensor(void)
{
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);
    IfxPort_setPinModeInput(IfxGpt120_CAPINA_P13_2_IN.pin.port, IfxGpt120_CAPINA_P13_2_IN.pin.pinIndex, IfxPort_InputMode_pullUp);
    IfxPort_setPinPadDriver(IfxGpt120_CAPINA_P13_2_IN.pin.port, IfxGpt120_CAPINA_P13_2_IN.pin.pinIndex, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(&IfxGpt120_T5EUDB_P10_1_IN, IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_T5_setCaptureTrigger(&MODULE_GPT120, IfxGpt12_CaptureTrigger_capin);
    IfxGpt12_T5_setCaptureTriggerMode(&MODULE_GPT120, IfxGpt12_CaptureTriggerMode_risingEdge);
    IfxGpt12_T5_enableClearTimer(&MODULE_GPT120, 1);
    IfxGpt12_T5_setCaptureTriggerEnable(&MODULE_GPT120, 1);
    IfxGpt12_T5_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T5_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);
    IfxGpt12_T5_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void Get_Speed(void)
{
    speed_Measured1 = (float)(uint16)MODULE_GPT120.CAPREL.U;
}
*/


//由speed_Target，speed_Measured得到speed_Output
void Cal_Speed_Output1(void)
{
    static int flag = 3;
    static float speed_Error[3];
    if (PID_mode1 == OPEN_LOOP1)
    {
        if (flag>=1)
        {
            flag--;
        }
        speed_Error[2] = speed_Error[1];
        speed_Error[1] = speed_Error[0];
        speed_Error[0] = speed_Target1 - speed_Measured1;

        speed_Output1 = speed_Target1;
    }
    else if (PID_mode1 == PID_CLOSED_LOOP1)
    {
        if (flag>=1)
        {
            flag--;
        }
        speed_Error[2] = speed_Error[1];
        speed_Error[1] = speed_Error[0];
        speed_Error[0] = speed_Target1 - speed_Measured1;
        if (flag == 0)
        {
//            flag++;
            if (speed_Error[0]<0.5 && speed_Error[0]>-0.5)
            {
                float delta_Speed = PID_KP1*(speed_Error[0]-speed_Error[1]) + PID_KD1*(speed_Error[0]-2*speed_Error[1]+speed_Error[2]);
                speed_Output1 = speed_Output1 + delta_Speed;
            }
            else
            {
                float delta_Speed = PID_KP1*(speed_Error[0]-speed_Error[1]) + PID_KI1*speed_Error[0] + PID_KD1*(speed_Error[0]-2*speed_Error[1]+speed_Error[2]);
                speed_Output1 = speed_Output1 + delta_Speed;
            }
        }
    }
    else if (PID_mode1 == FUZZY_PID_CLOSED_LOOP1)
    {
        speed_Output1 = fuzzy_pid_control(speed_Measured1, speed_Target1, pid_vector[0]);
    }
    else if (PID_mode1 == BANGBANG_CLOSED_LOOP1)
    {
        if (flag>=1)
        {
            flag--;
        }
        speed_Error[2] = speed_Error[1];
        speed_Error[1] = speed_Error[0];
        speed_Error[0] = speed_Target1 - speed_Measured1;

        if (speed_Measured1 > speed_Target1+BANGBANG_UP1)
        {
            speed_Output1 = 0.2f*SPEED_MIN1;//0.2f*SPEED_MIN1;
        }
        if  (speed_Measured1 < speed_Target1-BANGBANG_DOWN1)
        {
            speed_Output1 = 0.8f*SPEED_MAX1;
        }
    }

}



void Set_Speed1(void)
{
    uint32 duty;
    duty = (speed_Output1>=0?speed_Output1:(-speed_Output1))*700 +100;//地面开环用它
    duty = (uint32)(duty*Base_Volt/Real_Volt);
    if (duty>(uint32)(MOTOR_DUTY_MAX1))//保护电机
    {
        duty = (uint32)(MOTOR_DUTY_MAX1);
    }
    if (speed_Output1 == 0)
    {
        duty = 0;
    }
//    pwm_duty(ATOM0_CH6_P23_1, (uint32)(5000-(speed_Output1>0?1:(-1))*duty));

    static uint8 PWM_SetChannel = 0; //0表示0的端口，1表示duty的端口
    static uint8 flag_forward = 1;
    static uint8 flag_backward = 1;

    if (speed_Output1>=0 && flag_forward == 1)//只有改变方向为向前，才会执行
    {
        pwm_duty(ATOM0_CH6_P23_1, (uint32)(0));
        flag_forward = 0;
        flag_backward = 1;
        PWM_SetChannel = 0;
    }
    else if (speed_Output1<0 && flag_backward == 1)//只有改变方向为向后，才会执行
    {
        pwm_duty(ATOM1_CH5_P32_4, (uint32)(0));
        flag_forward = 1;
        flag_backward = 0;
        PWM_SetChannel = 0;
    }

    //只有PWM_SetChannel = 1才会执行
    if (speed_Output1>=0 && PWM_SetChannel == 1)
    {
        pwm_duty(ATOM1_CH5_P32_4, (uint32)(duty));
    }
    else if (speed_Output1<0 && PWM_SetChannel == 1)
    {
        pwm_duty(ATOM0_CH6_P23_1, (uint32)(duty));
    }

    if (PWM_SetChannel == 0)
    {
        PWM_SetChannel = 1;
    }

}

void Cal_Speed_Target1(void)
{
    ;//根据Col_Center和扫描范围search_Lines计算速度目标speed_Target，待完成
}

