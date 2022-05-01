#ifndef __UART_h__
#define __UART_h__

#define RECEIVE_LENGTH ((4+1+4 + 4+1+4 + 4+4+4 + 4+1+4 + 4+1+4 + 4+6+4 + 4+6+4 + 4+1+4 + 4+6+4 + 4+1+4)*2)
#define EMERGENCY_RECEIVE_LENGTH ((4+1+4 + 4+1+4)*2)
#define CACHE_LENGTH 0

#define BYTE0(x) (*((uint8*)(&(x))))
#define BYTE1(x) (*((uint8*)(&(x))+1))
#define BYTE2(x) (*((uint8*)(&(x))+2))
#define BYTE3(x) (*((uint8*)(&(x))+3))

#define FLOAT(x) (*((float*)(x)))
#define UINT32(x) (*((uint32*)(x)))

enum UARTstate
{
    Read,
    Send,
    Emergency_Read
};

extern uint8 data_Buffer[RECEIVE_LENGTH + CACHE_LENGTH];
extern uint8 *dat;
extern uint8 UART_Flag_RX;
extern uint8 UART_Flag_TX;
extern uint8 UART_EN;
extern uint8 data_Buffer_Shadow[RECEIVE_LENGTH];
extern uint8 UART_Flag_NO_IMAGE;

void My_Init_UART(void);
void UART(enum UARTstate state);


#endif
