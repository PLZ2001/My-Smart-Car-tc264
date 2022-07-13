/** 
* @brief        移植用的适配STM32的HAL库的驱动文件 非官方 自己适配的
* @author      WMD
* @date     2018年4月22日21:20:32
* @version  
* @par Copyright (c):  
*       WMD 
* @par 日志
*/  
#include "headfile.h"
#include "ST_I2C.h"
#include "vl53l0x_api.h"
//#include "cmsis_os.h"

/**
            * @name 自定义移植接口函数
            * @{
            */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8 index, uint8 *pdata, uint32 count)
{
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,count,pdata);
}
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8 index, uint8 *pdata, uint32 count)
{
	return ST_IICreadBytes(Dev->I2cDevAddr,index,count,pdata);
}
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8 index, uint8 data)
{
	return ST_IICwriteByte(Dev->I2cDevAddr,index,data);
}
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8 index, uint8 *data)
{
	return VL53L0X_ReadMulti(Dev,index,data,1);
}
///注意以下这些函数有可能会导致大小端不对
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8 index, uint16 data)
{
	uint8 tmp[2]={	((uint16)(data&0xff00)>>8) ,	((uint16)data&0x00ff)	};
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,2,tmp);
}
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8 index, uint32 data)
{
	uint8 tmp[4]={	((data&0xff000000)>>24),((data&0x00ff0000)>>16),((data&0x0000ff00)>>8),((data&0x000000ff))};
	return ST_IICwriteBytes(Dev->I2cDevAddr,index,4,tmp);
}
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8 index, uint16 *data)
{
	uint8 tmp[2];
	uint8* p=(uint8*)data;
	ST_IICreadBytes(Dev->I2cDevAddr,index,2,tmp);
	p[0]=tmp[1];
	p[1]=tmp[0];
	return 0;
}
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8 index, uint32 *data)
{
	uint8 tmp[4];
	uint8* p=(uint8*)data;
	ST_IICreadBytes(Dev->I2cDevAddr,index,4,tmp);
	p[0]=tmp[3];
	p[1]=tmp[2];
	p[2]=tmp[1];
	p[3]=tmp[0];
	return 0;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8 index, uint8 AndData, uint8 OrData)
{
	uint8 tmp;
	ST_IICreadBytes(Dev->I2cDevAddr,index,1,&tmp);
	tmp=(tmp & AndData) | OrData;
	return ST_IICwriteByte(Dev->I2cDevAddr,index,tmp);
}
//!该函数是等待用函数 要改成对应环境的Delay
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    systick_delay_ms(STM0, 50);
	return 0;
}
            /** @} */
//接下来是要引出的简化API函数
//!激光传感器的结构体
static VL53L0X_Dev_t TestDev_s;
static VL53L0X_DEV TestDev=&TestDev_s;
static VL53L0X_RangingMeasurementData_t TestData_s;
static VL53L0X_RangingMeasurementData_t* TestData=&TestData_s;

/** 
* @brief  激光传感器初始化
* @param void
* @retval  void
* @par 日志
*
*/
void VL53L0X_Init(void)
{
    simiic_init();
    systick_delay_ms(STM0, 10);  //上电延时


	TestDev->I2cDevAddr=0x52;
	VL53L0X_SetDeviceAddress(TestDev,0x52);//设置地址
	VL53L0X_SetPowerMode(TestDev,VL53L0X_POWERMODE_STANDBY_LEVEL2);//设定最不省电模式
    VL53L0X_SetDeviceMode(TestDev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//设定连续读取模式
    VL53L0X_SetInterMeasurementPeriodMilliSeconds(TestDev,10);//设定采样时间
	VL53L0X_DataInit(TestDev);
}
/** 
* @brief  获得激光传感器的距离
* @param void
* @retval  uint16 距离(单位mm)
* @note 如果不在任务中使用的话记得修改延时函数
* @par 日志
*
*/
uint16 VL53L0X_GetValue(void)
{
		VL53L0X_PerformSingleMeasurement(TestDev);//简单测量
		VL53L0X_GetRangingMeasurementData(TestDev,TestData);
        return TestData->RangeMilliMeter;
}


