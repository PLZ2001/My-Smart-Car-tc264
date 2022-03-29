#include "headfile.h"
#include "MOTOR2.h"
#include "IfxGpt12_reg.h"
#include "IfxGpt12.h"
#include "fuzzy_PID.h"//ģ��PID�㷨




//��Ҫ����ͨ�����ȥ�������ô������ı���
float speed_Measured2 = 0;//�����ٶȣ�m/s��


//��Ҫ����ͨ�Ŵ������ı�������������ִ�б������µĺ�����
float speed_Target2 = 0;//Ŀ���ٶȣ�m/s�������º���Set_Speed_Target2(uint8 val)


//��������
float speed_Output2 = 0;//����ٶȣ�m/s��
float PID_KP2=7.0f/1.0f;//7.0f/0.5f;//7.0f/3.0f;
float PID_KI2=0.04f;//0.04f;//0.01f;
float PID_KD2=0.0f;//0.0f;

enum PID_Mode2 PID_mode2 = PID_CLOSED_LOOP2;//PIDģʽѡ��

void My_Init_Motor2(void)
{
    gtm_pwm_init(ATOM1_CH0_P21_2, 12500, 5000);//����P21.2���PWM����Ƶ��12.5kHz��ռ�ձ�5000/10000������ֱ���������
    gtm_pwm_init(ATOM1_CH4_P22_3, 12500, 5000);//����P22.3���PWM����Ƶ��12.5kHz��ռ�ձ�5000/10000������ֱ���������
}

void My_Init_SpeedSensor2(void)
{
    gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDB_P33_5);
}

void UART_Speed2(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x12);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    uart_putchar(DEBUG_UART, (uint8)round((speed_Measured2 - SPEED_MIN2)/(SPEED_MAX2-SPEED_MIN2)*255));
    uart_putchar(DEBUG_UART, (uint8)round((speed_Output2 - SPEED_MIN2)/(SPEED_MAX2-SPEED_MIN2)*255));
    uart_putchar(DEBUG_UART, (uint8)round((speed_Target2 - SPEED_MIN2)/(SPEED_MAX2-SPEED_MIN2)*255));
    uart_putchar(DEBUG_UART, (uint8)round((pid_vector[0]->last_error - SPEED_MIN2)/(SPEED_MAX2-SPEED_MIN2)*255));
    uart_putchar(DEBUG_UART, (uint8)round((pid_vector[0]->current_error - SPEED_MIN2)/(SPEED_MAX2-SPEED_MIN2)*255));
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x12);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void Set_Speed_Target2(uint8 val)
{
    speed_Target2 = ((float)val) / 256 * (SPEED_MAX2-SPEED_MIN2)+SPEED_MIN2;//��Ϊ�ٶȷ�Χ�ǶԳƵģ���255���Գƣ����Խ���ʱ��256
}


void UART_PID2(void)
{
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x13);
    uart_putchar(DEBUG_UART,0x01);//��������ͷ
    int16 kp = (int16)round(1000*PID_KP2);
    int16 ki = (int16)round(1000*PID_KI2);
    int16 kd = (int16)round(1000*PID_KD2);
    uart_putchar(DEBUG_UART, kp>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kp&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, ki>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, ki&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kd>>8);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART, kd&0x00FF);//�ȴ���8λ���ٴ���8λ
    uart_putchar(DEBUG_UART,0x00);
    uart_putchar(DEBUG_UART,0xff);
    uart_putchar(DEBUG_UART,0x13);
    uart_putchar(DEBUG_UART,0x02);//��������β
}

void Set_PID2(uint8 v1, uint8 v2, uint8 v3, uint8 v4, uint8 v5, uint8 v6)
{
    PID_KP2 = (v1*256.0f+v2)/1000.0f;
    PID_KI2 = (v3*256.0f+v4)/1000.0f;
    PID_KD2 = (v5*256.0f+v6)/1000.0f;
}

void Get_Speed_perSPEED_MEASURING_PERIOD_ms2(void)
{
    uint16 count = gpt12_get(GPT12_T4);
    gpt12_clear(GPT12_T4);
    if (count <= 32767)
    {
        speed_Measured2 = -((float)count)*0.033*2*3.1415926/(1.0*SPEED_MEASURING_PERIOD_ms2/1000.0)/1024.0;
    }
    else
    {
        speed_Measured2 = -((float)count-65536)*0.033*2*3.1415926/(1.0*SPEED_MEASURING_PERIOD_ms2/1000.0)/1024.0;
    }
    // (����������/��������)/1024 = ת�٣�ת/s����(����������/��������)/1024*0.033*2*3.1415926 = �ٶȣ�m/s��
}


/*ͨ��T5��Captureģʽ������
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
    speed_Measured2 = (float)(uint16)MODULE_GPT120.CAPREL.U;
}
*/


//��speed_Target��speed_Measured�õ�speed_Output
void Cal_Speed_Output2(void)
{

    if (PID_mode2 == OPEN_LOOP2)
    {
        speed_Output2 = speed_Target2;
    }
    else if (PID_mode2 == PID_CLOSED_LOOP2)
    {
        static int flag = 3;
        flag--;
        static float speed_Error[3];
        speed_Error[2] = speed_Error[1];
        speed_Error[1] = speed_Error[0];
        speed_Error[0] = speed_Target2 - speed_Measured2;
//        if (speed_Error[0]<0.1 && speed_Error[0]>-0.1)
//        {
//            speed_Error[0] = 0;
//        }//������������ʽpid
        if (flag == 0)
        {
            flag++;
            float delta_Speed = PID_KP2*(speed_Error[0]-speed_Error[1]) + PID_KI2*speed_Error[0] + PID_KD2*(speed_Error[0]-2*speed_Error[1]+speed_Error[2]);
            speed_Output2 = speed_Output2 + delta_Speed;
        }
    }
    else if (PID_mode2 == FUZZY_PID_CLOSED_LOOP2)
    {
        speed_Output2 = fuzzy_pid_control(speed_Measured2, speed_Target2, pid_vector[0]);
    }

}



void Set_Speed2(void)
{
    uint32 duty;
    duty = (speed_Output2>0?speed_Output2:(-speed_Output2))*700 +100;//���濪������
    if (duty>MOTOR_DUTY_MAX2)//�������
    {
        duty = MOTOR_DUTY_MAX2;
    }
    if (speed_Output2 == 0)
    {
        duty = 0;
    }
    pwm_duty(ATOM1_CH0_P21_2, (uint32)(5000+(speed_Output2>0?1:(-1))*duty));
}

void Cal_Speed_Target2(void)
{
    ;//����Col_Center��ɨ�跶Χsearch_Lines�����ٶ�Ŀ��speed_Target�������
}

