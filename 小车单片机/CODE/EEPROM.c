#include "headfile.h"
#include "EEPROM.h"


void My_Init_EEPROM(void)
{

}

void EEPROM_Clear_ALl(void){
    for (uint32 i=0;i<=11;i++){
        if(flash_check(i, 0) || flash_check(i, 1) || flash_check(i, 2) || flash_check(i, 3))
        {
            eeprom_erase_sector(i);
        }
    }

}

void EEPROM_Write_Data(uint32 num, uint32 data)
{
    uint32 *buf = &data;
    //保存浮点数的时候，使用float_conversion_uint32宏定义进行转换后在保存
    //buf = &(float_conversion_uint32(data));
    uint32 sector_num = num / 1024 ;
    uint32 page_num = num % 1024;
    eeprom_page_program(sector_num, page_num, buf);
}

void EEPROM_Erase_Sector(uint32 sector_num){
    eeprom_erase_sector(sector_num);
}

