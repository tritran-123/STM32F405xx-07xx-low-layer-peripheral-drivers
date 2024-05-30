#ifndef STM32F407VG_CLOCK_CONFIG_H
#define STM32F407VG_CLOCK_CONFIG_H

#include "stm32f407vg.h"

/* Module Flash Memory Interface Register */
/* define information about each bit field */

/* Flash access control register (FLASH_ACR) - location bit field in register*/
#define DCEN 10   /* bit 10*/
#define ICEN 9    /* bit 9*/
#define PRFTEN 8  /* bit 8*/
#define LATENCY 0 /* bit 0*/
/* Status in bit field LATENCY */
#define LATENCY_Zero_Wait_State 0
#define LATENCY_One_Wait_State 1
#define LATENCY_Two_Wait_State 2
#define LATENCY_Three_Wait_State 3
#define LATENCY_Four_Wait_State 4
#define LATENCY_Five_Wait_State 5
#define LATENCY_Six_Wait_State 6
#define LATENCY_Seven_Wait_State 7

/* Module PWR - Power Controller Register */
/* PWR power control register (PWR_CR)  - location bit field in register*/
#define VOS 14 /* bit 14 */
/* Status in bit field VOS */
#define PWR_Scale_1_Mode 1 /* default value at reset */

/* Module RCC */
/* RCC (RCC_APB1ENR)- location bit field in register */
#define PWREN 28 /* bit 28 */

/* RCC clock control register (RCC_CR)- location bit field in register */
#define HSEON 16  /*bit 16*/
#define HSERDY 17 /* bit 17*/
#define PLLON 24  /* bit 24*/
#define PLLRDY 25 /* bit 25*/
/* RCC PLL configuration register (RCC_PLLCFGR) - location bit field in register*/
#define PLLM 0  /* bit 0 */
#define PLLN 6  /* bit 6*/
#define PLLP 16 /* bit 16*/
#define PLLSRC 22 /*bit 22*/
/* RCC clock configuration register (RCC_CFGR)- location bit field in register */
#define SW 0     /* bit 0*/
#define SWS 2    /* bit 2*/
#define HPRE 4   /* AHP Prescaler - bit 4*/
#define PPRE1 10 /* APB1 prescaler - bit 10*/
#define PPRE2 13 /* APB2 prescalerbit 17*/
/* Status in bit field  HPRE: AHB prescaler*/
#define AHB_Prescaler_Not_Div 0
/*  SW: System clock switch and SWS  */
#define SW_HSI_Selected 0
#define SW_HSE_Selected 1
#define SW_PLL_Selected 2
#define SW_Not_Allow 3

#define SWS_HSI_Already_Set 0
#define SWS_HSE_Already_Set 4
#define SWS_PLL_Already_Set 8
#define SWS_Not_Application 12

/* Status in PPRE1: APB Low speed prescaler (APB1) - APB2 - AHP */
#define PPRE1_AHB_Div_4 4
#define PPRE2_AHB_Div_2 2
#define HPRE_Sys_Not_Div 0
/* Status in Main PLL (PLLM-PLLN-PLLP) - PLLSRC */
#define PLLM_Div_4  4
#define PLLN_Mul_168 168
#define PLLP_Div_2  2
#define PLLSRC_HSI_Selected 0
#define PLLSRC_HSE_Selected 1

/* prototype function */
void SYS_Config_Clock_Tree(void);

#endif
