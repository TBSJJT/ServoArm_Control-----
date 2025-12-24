#include "stm32f10x.h"                  // Device header

uint32_t MyFlash_ReadWord(uint32_t Address)//读一个字
{
	return *((__IO uint32_t *)(Address));
}

uint16_t MyFlash_ReadHalfWord(uint32_t Address)//读半字
{
	return *((__IO uint16_t *)(Address));
}

uint8_t MyFlash_ReadByte(uint32_t Address)//读字节
{
	return *((__IO uint8_t *)(Address));
}

void MyFlash_ERASEALLPAGE(void)//全擦除
{
	FLASH_Unlock();//解锁
	FLASH_EraseAllPages();//擦除
	FLASH_Lock();//上锁
}

void MyFlash_ERASEPAGE(uint32_t PageAddress)//页擦除
{
	FLASH_Unlock();//解锁
	FLASH_ErasePage(PageAddress);//擦除
	FLASH_Lock();//上锁
}

void MyFlash_ProgramWord(uint32_t Address,uint32_t Data)//写一个字
{
	FLASH_Unlock();//解锁
	FLASH_ProgramWord(Address, Data);
	FLASH_Lock();//上锁
}

void MyFlash_ProgramHalfWord(uint32_t Address,uint16_t Data)//写ban字
{
	FLASH_Unlock();//解锁
	FLASH_ProgramHalfWord(Address,Data);
	FLASH_Lock();//上锁
}
