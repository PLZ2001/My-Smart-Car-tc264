#ifndef __WIFI_h__
#define __WIFI_h__

#define BYTE0(x) (*((uint8*)(&(x))))
#define BYTE1(x) (*((uint8*)(&(x))+1))
#define BYTE2(x) (*((uint8*)(&(x))+2))
#define BYTE3(x) (*((uint8*)(&(x))+3))

#define FLOAT(x) (*((float*)(x)))
#define UINT32(x) (*((uint32*)(x)))

void My_Init_Wifi(void);
void Send_with_Wifi(void);
void Send_Begin(void);
void Send_Variable(void);
void Send_Image(uint8* image_Address, uint8 image_Width, uint8 image_Height, uint8 processed_Width, uint8 processed_Height);
uint16 Get_Edge(uint8* image_Address, uint8 image_Width, uint8 image_Height, uint8 processed_Width, uint8 processed_Height);






#endif
