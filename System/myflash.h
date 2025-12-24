#ifndef __MYFLASH_H
#define __MYFLASH_H

uint32_t MyFlash_ReadWord(uint32_t Address);//¶ÁÒ»¸ö×Ö
uint16_t MyFlash_ReadHalfWord(uint32_t Address);//¶Á°ë×Ö
uint8_t MyFlash_ReadByte(uint32_t Address);//¶Á×Ö½Ú
void MyFlash_ERASEALLPAGE(void);//È«²Á³ý
void MyFlash_ERASEPAGE(uint32_t PageAddress);//ye²Á³ý
void MyFlash_ProgramWord(uint32_t Address,uint32_t Data);//Ð´Ò»¸ö×Ö
void MyFlash_ProgramHalfWord(uint32_t Address,uint16_t Data);//Ð´ban×Ö

#endif
