#ifndef __IOI2C_H
#define __IOI2C_H
//#include "stm32f4xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif
//IO鍙ｆ搷浣滃畯瀹氫箟
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

/**
            * @name 闇�瑕佽嚜琛屾洿鏀圭殑IIC_GPIO瀹氫箟
            * @{
            */
#define SCL_H         gpio_set (SEEKFREE_SCL, 1)
#define SCL_L         gpio_set (SEEKFREE_SCL, 0)

#define SDA_H         gpio_set (SEEKFREE_SDA, 1)
#define SDA_L         gpio_set (SEEKFREE_SDA, 0)

#define SCL_read      gpio_get (SEEKFREE_SCL)
#define SDA_read      gpio_get (SEEKFREE_SDA)
            /** @} */
extern void ST_IIC_Init(void);                //<鍒濆鍖朓IC鐨処O鍙�


extern unsigned char ST_IIC_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
extern unsigned char ST_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
extern uint8 ST_IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data);
extern uint8 ST_IICwriteBit(uint8 dev,uint8 reg,uint8 bitNum,uint8 data);
extern uint8 ST_IICreadBytes(uint8 SlaveAddress,uint8 REG_Address,uint8 len,uint8 *data);
#ifdef __cplusplus
}
#endif
#endif

//------------------End of File----------------------------
