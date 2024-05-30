#ifndef STM32F407VG_FLASH_H
#define STM32F407VG_FLASH_H

#include "stm32f407vg.h"

/* define each location bit field in register */
/* Flash key register (FLASH_KEYR) */
#define FLASH_KEYR_KEY_15_0    0
#define FLASH_KEYR_KEY_31_16   16
/* Flash status register (FLASH_SR) for STM32F405xx/07xx  */
#define FLASH_SR_EOP        0
#define FLASH_SR_OPERR      1
#define FLASH_SR_WRPERR     4
#define FLASH_SR_PGAERR     5
#define FLASH_SR_PGPERR     6
#define FLASH_SR_PGSERR     7
#define FLASH_SR_BSY        16  
/*  Flash control register (FLASH_CR) for STM32F405xx/07xx */
#define FLASH_CR_PG         0
#define FLASH_CR_SER        1
#define FLASH_CR_MER        2
#define FLASH_CR_SNB_0_3    3
#define FLASH_CR_PSIZE_1_0  8
#define FLASH_CR_STRT       16
#define FLASH_CR_EOPIE      24
#define FLASH_CR_ERRIE      25
#define FLASH_CR_LOCK       31

/* define information about status each bit field */
/* Key */
#define FLASH_KEY1      0x45670123 
#define FLASH_KEY2      0xCDEF89AB
/* Sector Erase */
#define FLASH_Erase_AllSector   12
#define FLASH_Erase_SectorNo0   0
#define FLASH_Erase_SectorNo1   1
#define FLASH_Erase_SectorNo2   2
#define FLASH_Erase_SectorNo3   3
#define FLASH_Erase_SectorNo4   4
#define FLASH_Erase_SectorNo5   5
#define FLASH_Erase_SectorNo6   6
#define FLASH_Erase_SectorNo7   7
#define FLASH_Erase_SectorNo8   8
#define FLASH_Erase_SectorNo9   9
#define FLASH_Erase_SectorNo10  10
#define FLASH_Erase_SectorNo11  11
/* Program size */
#define FLASH_PSIZE_x32     2



/*prototype func*/ 
static void FLASH_Unlock(void);
static void FLASH_Lock(void);
void FLASH_EraseSector(uint8_t EraseSectorNo);
void FLASH_ProgramWord(uint32_t AddressToWrite,uint8_t ProgramSizeData,uint32_t Data);





#endif
