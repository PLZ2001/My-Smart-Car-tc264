#include "headfile.h"
#include "ST_I2C.h"



static uint8 ST_IIC_Start(void);
static void ST_IIC_Stop(void);	  		
static void ST_IIC_Send_Byte(uint8 txd);
static uint8 ST_IIC_Read_Byte(void);
static uint8 ST_IIC_Wait_Ack(void);
static void ST_IIC_Ack(void);				
static void ST_IIC_NAck(void);		




static void IIC_delay(void)
{
    simiic_delay();
}


/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		void ST_IIC_Start(void)
*鍔熴��銆�鑳�:		浜х敓IIC璧峰淇″彿
*******************************************************************************/
static uint8 ST_IIC_Start(void)
{
    simiic_start();
	return TRUE;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		void ST_IIC_Stop(void)
*鍔熴��銆�鑳�:	    //浜х敓IIC鍋滄淇″彿
*******************************************************************************/
static void ST_IIC_Stop(void)
{
    simiic_stop();
}
static int sccb_waitack(void)
{
    SCL_L;
    gpio_dir (SEEKFREE_SDA, GPI, NO_PULL);
    simiic_delay();

    SCL_H;
    simiic_delay();

    if(SDA_read)           //应答为高电平，异常，通信失败
    {
        gpio_dir (SEEKFREE_SDA, GPO, PUSHPULL);
        SCL_L;
        return 0;
    }
    gpio_dir (SEEKFREE_SDA, GPO, PUSHPULL);
    SCL_L;
    simiic_delay();
    return 1;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 ST_IIC_Wait_Ack(void)
*鍔熴��銆�鑳�:	    绛夊緟搴旂瓟淇″彿鍒版潵
//杩斿洖鍊硷細1锛屾帴鏀跺簲绛斿け璐�
//        0锛屾帴鏀跺簲绛旀垚鍔�
*******************************************************************************/
static uint8 ST_IIC_Wait_Ack(void)
{
	return sccb_waitack();
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		void ST_IIC_Ack(void)
*鍔熴��銆�鑳�:	    浜х敓ACK搴旂瓟
*******************************************************************************/
static void ST_IIC_Ack(void)
{
    simiic_sendack(1);
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		void ST_IIC_NAck(void)
*鍔熴��銆�鑳�:	    浜х敓NACK搴旂瓟
*******************************************************************************/
static void ST_IIC_NAck(void)
{
    simiic_sendack(0);
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		void ST_IIC_Send_Byte(uint8 txd)
*鍔熴��銆�鑳�:	    IIC鍙戦�佷竴涓瓧鑺�
*******************************************************************************/
static void ST_IIC_Send_Byte(uint8 SendByte)
{
    uint8 i=8;
    while(i--)
    {
			SCL_L;
			IIC_delay();
			if(SendByte&0x80)
				SDA_H;
			else
				SDA_L;
			SendByte<<=1;
			IIC_delay();
			SCL_H;
			IIC_delay();
    }
    SCL_L;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 ST_IIC_Read_Byte(unsigned char ack)
*鍔熴��銆�鑳�:	    //璇�1涓插瓧鑺傦紝ack=1鏃讹紝鍙戦�丄CK锛宎ck=0锛屽彂閫乶ACK
*******************************************************************************/
static unsigned char ST_IIC_Read_Byte(void)  
{
    uint8 i=8;
    uint8 ReceiveByte=0;

    SDA_H;
    while(i--)
    {
			ReceiveByte<<=1;
			SCL_L;
			IIC_delay();
			SCL_H;
			IIC_delay();
			if(SDA_read)
			{
				ReceiveByte|=0x01;
			}
    }
    SCL_L;
    return ReceiveByte;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		unsigned char IIC_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*鍔熴��銆�鑳�:	    璇诲彇鎸囧畾璁惧 鎸囧畾瀵勫瓨鍣ㄧ殑涓�涓��
杈撳叆	I2C_Addr  鐩爣璁惧鍦板潃
		addr	   瀵勫瓨鍣ㄥ湴鍧�
杩斿洖   璇诲嚭鏉ョ殑鍊�
*******************************************************************************/
uint8 IIC_ReadOneByte(uint8 SlaveAddress,uint8 REG_Address)
{
		if(!ST_IIC_Start())
			return FALSE;
    ST_IIC_Send_Byte(SlaveAddress);
    if(!ST_IIC_Wait_Ack())
		{
			ST_IIC_Stop();
			return FALSE;
		}
    ST_IIC_Send_Byte((uint8) REG_Address);
    ST_IIC_Wait_Ack();
    ST_IIC_Start();
    ST_IIC_Send_Byte(SlaveAddress+1);
    ST_IIC_Wait_Ack();

		ST_IIC_Read_Byte();
    ST_IIC_NAck();
    ST_IIC_Stop();
    //return TRUE;
		return 0;

}


/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 IICreadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data)
*鍔熴��銆�鑳�:	    璇诲彇鎸囧畾璁惧 鎸囧畾瀵勫瓨鍣ㄧ殑 length涓��
杈撳叆	dev  鐩爣璁惧鍦板潃
		reg	  瀵勫瓨鍣ㄥ湴鍧�
		length 瑕佽鐨勫瓧鑺傛暟
		*data  璇诲嚭鐨勬暟鎹皢瑕佸瓨鏀剧殑鎸囬拡
杩斿洖   璇诲嚭鏉ョ殑瀛楄妭鏁伴噺
*******************************************************************************/
uint8 ST_IICreadBytes(uint8 SlaveAddress,uint8 REG_Address,uint8 len,uint8 *data)
{
		uint8 i = 0;
		if(!ST_IIC_Start())
			return FALSE;
    ST_IIC_Send_Byte(SlaveAddress); 
    if(!ST_IIC_Wait_Ack())
		{
			ST_IIC_Stop();
			return FALSE;
		}
    ST_IIC_Send_Byte((uint8) REG_Address);
    ST_IIC_Wait_Ack();
    ST_IIC_Start();
    ST_IIC_Send_Byte(SlaveAddress+1);
    ST_IIC_Wait_Ack();

		for(i = 0;i<len;i++)
		{
			if(i != (len -1))
			{
				data[i]= ST_IIC_Read_Byte();
				ST_IIC_Ack();
			}
			else
			{
				data[i]= ST_IIC_Read_Byte();
				ST_IIC_NAck();
			}
		}
		ST_IIC_Stop();
    //return TRUE;
		return 0;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data)
*鍔熴��銆�鑳�:	    灏嗗涓瓧鑺傚啓鍏ユ寚瀹氳澶� 鎸囧畾瀵勫瓨鍣�
杈撳叆	dev  鐩爣璁惧鍦板潃
		reg	  瀵勫瓨鍣ㄥ湴鍧�
		length 瑕佸啓鐨勫瓧鑺傛暟
		*data  灏嗚鍐欑殑鏁版嵁鐨勯鍦板潃
杩斿洖   杩斿洖鏄惁鎴愬姛
*******************************************************************************/
uint8 ST_IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data)
{

 	uint8 count = 0;
	ST_IIC_Start();
	ST_IIC_Send_Byte(dev);	   
	ST_IIC_Wait_Ack();
	ST_IIC_Send_Byte(reg);   
    ST_IIC_Wait_Ack();
	for(count=0;count<length;count++)
	{
		ST_IIC_Send_Byte(data[count]);
		ST_IIC_Wait_Ack();
	 }
	ST_IIC_Stop();

    return 0; //status == 0;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 IICreadByte(uint8 dev, uint8 reg, uint8 *data)
*鍔熴��銆�鑳�:	    璇诲彇鎸囧畾璁惧 鎸囧畾瀵勫瓨鍣ㄧ殑涓�涓��
杈撳叆	dev  鐩爣璁惧鍦板潃
		reg	   瀵勫瓨鍣ㄥ湴鍧�
		*data  璇诲嚭鐨勬暟鎹皢瑕佸瓨鏀剧殑鍦板潃
杩斿洖   1
*******************************************************************************/
uint8 ST_IICreadByte(uint8 dev, uint8 reg, uint8 *data)
{
	*data=IIC_ReadOneByte(dev, reg);
    return 0;
}

/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*鍔熴��銆�鑳�:	    鍐欏叆鎸囧畾璁惧 鎸囧畾瀵勫瓨鍣ㄤ竴涓瓧鑺�
杈撳叆	dev  鐩爣璁惧鍦板潃
		reg	   瀵勫瓨鍣ㄥ湴鍧�
		data  灏嗚鍐欏叆鐨勫瓧鑺�
杩斿洖   1
*******************************************************************************/
uint8 ST_IICwriteByte(uint8 dev, uint8 reg, uint8 data)
{
    return ST_IICwriteBytes(dev, reg, 1, &data);
}


/**************************瀹炵幇鍑芥暟********************************************
*鍑芥暟鍘熷瀷:		uint8 IICwriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
*鍔熴��銆�鑳�:	    璇� 淇敼 鍐� 鎸囧畾璁惧 鎸囧畾瀵勫瓨鍣ㄤ竴涓瓧鑺� 涓殑1涓綅
杈撳叆	dev  鐩爣璁惧鍦板潃
		reg	   瀵勫瓨鍣ㄥ湴鍧�
		bitNum  瑕佷慨鏀圭洰鏍囧瓧鑺傜殑bitNum浣�
		data  涓�0 鏃讹紝鐩爣浣嶅皢琚竻0 鍚﹀垯灏嗚缃綅
杩斿洖   鎴愬姛 涓�1
 		澶辫触涓�0
*******************************************************************************/
uint8 ST_IICwriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
{
    uint8 b;
    ST_IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return ST_IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
