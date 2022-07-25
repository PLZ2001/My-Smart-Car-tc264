/** 
* @brief        绉绘鐢ㄧ殑閫傞厤STM32鐨凥AL搴撶殑椹卞姩鏂囦欢 闈炲畼鏂� 鑷繁閫傞厤鐨�
* @author      WMD
* @date     2018骞�4鏈�22鏃�21:20:32
* @version  
* @par Copyright (c):  
*       WMD 
* @par 鏃ュ織
*/  
#include "headfile.h"
#include "ST_I2C.h"
#include "vl53l0x_api.h"
#include "ICM.h"
//#include "cmsis_os.h"

/**
            * @name 鑷畾涔夌Щ妞嶆帴鍙ｅ嚱鏁�
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
///娉ㄦ剰浠ヤ笅杩欎簺鍑芥暟鏈夊彲鑳戒細瀵艰嚧澶у皬绔笉瀵�
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
//!璇ュ嚱鏁版槸绛夊緟鐢ㄥ嚱鏁� 瑕佹敼鎴愬搴旂幆澧冪殑Delay
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    systick_delay_ms(STM0, 50);
	return 0;
}
            /** @} */
//鎺ヤ笅鏉ユ槸瑕佸紩鍑虹殑绠�鍖朅PI鍑芥暟
//!婵�鍏変紶鎰熷櫒鐨勭粨鏋勪綋
static VL53L0X_Dev_t TestDev_s;
static VL53L0X_DEV TestDev=&TestDev_s;
static VL53L0X_RangingMeasurementData_t TestData_s;
static VL53L0X_RangingMeasurementData_t* TestData=&TestData_s;

/** 
* @brief  婵�鍏変紶鎰熷櫒鍒濆鍖�
* @param void
* @retval  void
* @par 鏃ュ織
*
*/
void VL53L0X_Init(void)
{
    simiic_init();
    systick_delay_ms(STM0, 10);  //涓婄數寤舵椂


	TestDev->I2cDevAddr=0x52;
	VL53L0X_SetDeviceAddress(TestDev,0x52);//璁剧疆鍦板潃
	VL53L0X_SetPowerMode(TestDev,VL53L0X_POWERMODE_STANDBY_LEVEL2);//璁惧畾鏈�涓嶇渷鐢垫ā寮�
    VL53L0X_SetDeviceMode(TestDev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//璁惧畾杩炵画璇诲彇妯″紡
    VL53L0X_SetInterMeasurementPeriodMilliSeconds(TestDev,10);//璁惧畾閲囨牱鏃堕棿
	VL53L0X_DataInit(TestDev);

	flag_for_ICM_Init = 1;
}
/** 
* @brief  鑾峰緱婵�鍏変紶鎰熷櫒鐨勮窛绂�
* @param void
* @retval  uint16 璺濈(鍗曚綅mm)
* @note 濡傛灉涓嶅湪浠诲姟涓娇鐢ㄧ殑璇濊寰椾慨鏀瑰欢鏃跺嚱鏁�
* @par 鏃ュ織
*
*/
uint16 VL53L0X_GetValue(void)
{
		VL53L0X_PerformSingleMeasurement(TestDev);//绠�鍗曟祴閲�
		VL53L0X_GetRangingMeasurementData(TestDev,TestData);
        return TestData->RangeMilliMeter;
//		return TestData->RangeStatus;
}


