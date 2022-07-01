#ifndef __EEPROM_h__
#define __EEPROM_h__


#define     EEPROM_Read_Data(num,type)      (*(type *)((uint32)((EEPROM_BASE_ADDR + (num / 1024)*EEPROM_SECTOR_SIZE) + ((num % 1024)*8))))

#define ID_STEERING_DUTY_CENTER 0


#endif
