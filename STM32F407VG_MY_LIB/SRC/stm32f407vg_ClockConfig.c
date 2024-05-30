#include "stm32f407vg_ClockConfig.h"
/* this is just a temp function to set max clock in APBx Bus so if have any update with another clock, 
Please adjust funtion USART_SetBaudRate() in module USART */
void SYS_Config_Clock_Tree(void)
{
    /* Enable HSE and wait for the HSERDY to become ready */
    RCC->CR |= (Enable << HSEON);
    while (!(RCC->CR & (FLAG_ALREADY_SET << HSERDY)));

    /* Set the PWR Enable clock and Voltage Regulator */
    RCC->APB1ENR |= (Enable << PWREN);
    PWR->CR |= (PWR_Scale_1_Mode << VOS); /*  default value at reset */

    /* Config Flash FREFETCH and the LAtency relate setting */
    FlashInterface->ACR |= (Enable << PRFTEN); /* Prefetch is enabled */
    FlashInterface->ACR |= (Enable << ICEN);   /* Instruction cache is enabled*/
    FlashInterface->ACR |= (Enable << DCEN);   /* Data cache is enabled */
    FlashInterface->ACR |= (LATENCY_Five_Wait_State << LATENCY);


    /* Config Prescaler HCLK , PCLK1 ,PCLK2 */
    RCC->CFGR |= (HPRE_Sys_Not_Div << HPRE); 
    RCC->CFGR |= (PPRE1_AHB_Div_4 << PPRE1);
    RCC->CFGR |= (PPRE2_AHB_Div_2 << PPRE2);

    /* Config the MAIN PLL and select source clock entry (HSI or HSE )*/
    RCC->PLLCFGR = Reset; /*Reset value: 0x2400 3010*/

    RCC->PLLCFGR |= (PLLM_Div_4 << PLLM);
    RCC->PLLCFGR |= (PLLN_Mul_168 << PLLN);
    RCC->PLLCFGR |= (PLLP_Div_2 << PLLP);

    RCC->PLLCFGR |= (PLLSRC_HSE_Selected << PLLSRC);

    /* Enable Main PLL and wait it become ready */
    RCC->CR |= (Enable << PLLON);
    while(!(RCC->CR & (FLAG_ALREADY_SET << PLLRDY)));

    /* Select Clock Source (SW) and wait it to be set */
    RCC->CFGR |= (SW_PLL_Selected << SW);
    while((RCC->CFGR & (3 << SWS)) != SWS_PLL_Already_Set);

}
