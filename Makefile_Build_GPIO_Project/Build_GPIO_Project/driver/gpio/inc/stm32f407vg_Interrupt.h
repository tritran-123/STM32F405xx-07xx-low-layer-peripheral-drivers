
#ifndef STM32F407VG_Interrupt_H
#define STM32F407VG_Interrupt_H

#include "stm32f407vg.h"

/* macro bit enbale clock for module syscfg in (RCC_APB2ENR) */
#define SYSCFGEN 14

/* Function enable & disable clock for SYSCFG */
#define SYSCFG_Enable_Clock() ((RCC->APB2ENR) |= (1 << SYSCFGEN))
#define SYSCFG_Disable_Clock() ((RCC->APB2ENR) &= ~(1 << SYSCFGEN))

/* if-else to determine value of 4bit corresponds to 10 ports (A,B..k) in (SYSCFG_EXTICR(1-4))*/
#define Corresponds_Value_For_Each_Port(Portx) ((Portx == GPIOA) ? 0x0 : (Portx == GPIOB) ? 0x1 : (Portx == GPIOC) ? 0x2 :(Portx == GPIOD) ? 0x3 :(Portx == GPIOE) ? 0x4 :(Portx == GPIOF) ? 0x5 :(Portx == GPIOG) ? 0x6 :(Portx == GPIOH) ? 0x7 :(Portx == GPIOI) ? 0x8 :0)

/* define IRQNumber EXTI */
#define  IRQNumber_EXTI0        6
#define  IRQNumber_EXTI1        7
#define  IRQNumber_EXTI2        8
#define  IRQNumber_EXTI3        9
#define  IRQNumber_EXTI4        10
#define  IRQNumber_EXTI9_5      23
#define  IRQNumber_EXTI15_10    40
/* define IRQNumber USART */
#define  IRQNumber_USART1        37
#define  IRQNumber_USART2        38
#define  IRQNumber_USART3        39
#define  IRQNumber_UART4         52
#define  IRQNumber_UART5         53
#define  IRQNumber_USART6        71
/* define IRQNumber SPI */
#define  IRQNumber_SPI1        35
#define  IRQNumber_SPI2        36
#define  IRQNumber_SPI3        51
/* define IRQNumber I2C */
#define  IRQNumber_I2C1_EV        31
#define  IRQNumber_I2C1_ER        32
#define  IRQNumber_I2C2_EV        33
#define  IRQNumber_I2C2_ER        34
#define  IRQNumber_I2C3_EV        72
#define  IRQNumber_I2C3_ER        73






/* prototype funtion */
void NVIC_SetPriority (uint8_t IRQNumber, uint8_t Priority);
void NVIC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis);

/*Systick timer 24bit in core */
#define TICK_NUMBER_IN_1MS 15999 /* use processorcore = 16Mhz -> need 15999 or 0x2903f tick*/
void SysTick_Initialize(uint32_t nTick);
void SysTick_Handler(void);
void Delay(uint32_t nTime);

#endif
