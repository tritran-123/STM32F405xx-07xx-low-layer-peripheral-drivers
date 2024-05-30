#include "stm32f407vg_flash_driver.h"

void FLASH_Unlock(void)
{
    /* Default FLASH alway lock -> check bit Lock in  (FLASH_CR) */
    if (FlashInterface->CR & (Enable << FLASH_CR_LOCK))
    {
        /* Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
        Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)*/
        FlashInterface->KEYR = ((FLASH_KEY1 << FLASH_KEYR_KEY_15_0) | (FLASH_KEY2 << FLASH_KEYR_KEY_31_16));
    }
}

void FLASH_Lock(void)
{
    FlashInterface->CR |= (Enable << FLASH_CR_LOCK);
}

void FLASH_Erase(uint8_t EraseSectorNo)
{
    /*Unlock Flash*/
    FLASH_Unlock();

    /*  Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register*/
    while ((FlashInterface->SR & (Set << FLASH_SR_BSY)) == 1)
        ;
    /* Set the SER bit and select the sector out of the 12 sectors */
    if (EraseSectorNo == FLASH_Erase_AllSector)
    {
        /* Set MASS Erase in CR */
        FlashInterface->CR |= (Enable << FLASH_CR_MER);
    }
    else
    {
        /* Set the SER bit and select the sector out of the 12 sectors (for STM32F405xx/07xx and
STM32F415xx/17xx) and out of 24 (for STM32F42xxx and STM32F43xxx) in the main
memory block you wish to erase (SNB) in the FLASH_CR register*/
        FlashInterface->CR |= ((Enable << FLASH_CR_SER) | (EraseSectorNo << FLASH_CR_SNB_0_3));
    }
    /*  Set the STRT bit in the FLASH_CR register */
    FlashInterface->CR |= (Enable << FLASH_CR_STRT);
    /* Wait for the BSY bit to be cleared */
    while ((FlashInterface->SR & (Set << FLASH_SR_BSY)) == 1)
        ;

    /* Lock FLash */
    FLASH_Lock();
}

void FLASH_ProgramWord(uint32_t AddressToWrite, uint8_t ProgramSizeData, uint32_t Data)
{
    /*Unlock Flash*/
    FLASH_Unlock();

    /*  Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register*/
    while ((FlashInterface->SR & (Set << FLASH_SR_BSY)) == 1)
        ;
    /*  Set the PG bit in the FLASH_CR register */
    FlashInterface->CR |= (Enable << FLASH_CR_PG);
    /*  Perform the data write operation(s) to the desired memory address */
    FlashInterface->CR |= (ProgramSizeData << FLASH_CR_PSIZE_1_0);
    *((uint32_t *)(AddressToWrite)) = Data;
    /* Wait for the BSY bit to be cleared */
    while ((FlashInterface->SR & (Set << FLASH_SR_BSY)) == 1)
        ;

    /* Lock FLash */
    FLASH_Lock();
}