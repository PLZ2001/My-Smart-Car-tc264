#include "headfile.h"
#include "STEERING.h"
#include "EEPROM.h"


void EEPROM_Clear_ALl(void){
    for (uint32 i=0;i<=11;i++){
            eeprom_erase_sector(i);
    }

}

void EEPROM_Write_Data(uint32 num, uint32 *buf){
    uint32 sector_num = num / 1024 ;
    uint32 page_num = num % 1024;
    eeprom_page_program(sector_num, page_num, buf);
}


void EEPROM_Erase_Sector(uint32 sector_num){
    eeprom_erase_sector(sector_num);
}

