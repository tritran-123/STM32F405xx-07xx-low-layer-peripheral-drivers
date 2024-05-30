
#ifndef STM32F407VG_H
#define STM32F407VG_H

#include <stdint.h>
#include <stdio.h>
#include <math.h> /* function (round) to extract decimal part in driver uart */

/* define keyword in C */
#define _vo volatile
/* define Generic keyword in C */
#define Enable 1
#define Disable 0
#define Set 1
#define Reset 0
#define High 1
#define Low 0
#define FLAG_ALREADY_SET 1
#define FLAG_ALREADY_RESET 0
/*--------------------------------------------------------------------------------------------------------------------------*/
/* define base address 's modules */

/* define Bus in MemoryMap */
#define APB1_Base_Address 0x40000000
#define APB2_Base_Address 0x40010000
#define AHB1_Base_Address 0x40020000
#define AHB2_Base_Address 0x50000000

                        /* define peripherals hang on Bus AHB1 */
/* GPIO */
#define GPIOA_Base_Address (AHB1_Base_Address + 0x00000000)
#define GPIOB_Base_Address (AHB1_Base_Address + 0x00000400)
#define GPIOC_Base_Address (AHB1_Base_Address + 0x00000800)
#define GPIOD_Base_Address (AHB1_Base_Address + 0x00000C00)
#define GPIOE_Base_Address (AHB1_Base_Address + 0x00001000)
#define GPIOF_Base_Address (AHB1_Base_Address + 0x00001400)
#define GPIOG_Base_Address (AHB1_Base_Address + 0x00001800)
#define GPIOH_Base_Address (AHB1_Base_Address + 0x00001C00)
#define GPIOI_Base_Address (AHB1_Base_Address + 0x00002000)
#define GPIOJ_Base_Address (AHB1_Base_Address + 0x00002400)
#define GPIOK_Base_Address (AHB1_Base_Address + 0x00002800)
/* RCC */
#define RCC_Base_Address (AHB1_Base_Address + 0x00003800)
/* Flash interface register */
#define FlashInterface_Base_Address (AHB1_Base_Address + 0x00003C00)

                        /* define peripherals hang on Bus APB2 */
/*  System configuration controller (SYSCFG) - Bus APB2 */
#define SYSCFG_Base_Address (APB2_Base_Address + 0x00003800)
/* External interrupt/event  (EXTI - Bus APB2) */
#define EXTI_Base_Address (APB2_Base_Address + 0x00003C00)
/* SPI1 + SPI4 + SPI5 + SPI6 */
#define SPI1_Base_Address (APB2_Base_Address + 0x00003000)
#define SPI4_Base_Address (APB2_Base_Address + 0x00003400)
#define SPI5_Base_Address (APB2_Base_Address + 0x00005000)
#define SPI6_Base_Address (APB2_Base_Address + 0x00005400)
/* USART6 + USART1 */
#define USART1_Base_Address (APB2_Base_Address + 0x00001000)
#define USART6_Base_Address (APB2_Base_Address + 0x00001400)


                        /* define peripherals hang on Bus APB1 */
/* SPI2 + SPI3 */
#define SPI2_Base_Address (APB1_Base_Address + 0x00003800)
#define SPI3_Base_Address (APB1_Base_Address + 0x00003C00)
/* PWR - Power controller*/
#define PWR_Base_Address (APB1_Base_Address + 0x00007000)
/* USART2 + USART3 + UART4 + UART5 + UART7 + UART8  */
#define USART2_Base_Address (APB1_Base_Address + 0x00004400)
#define USART3_Base_Address (APB1_Base_Address + 0x00004800)
#define UART4_Base_Address (APB1_Base_Address + 0x00004C00)
#define UART5_Base_Address (APB1_Base_Address + 0x00005000)
#define UART7_Base_Address (APB1_Base_Address + 0x00007800)
#define UART8_Base_Address (APB1_Base_Address + 0x00007C00)
/* I2C1 + I2C2 + I2C3 */
#define I2C1_Base_Address (APB1_Base_Address + 0x00005400)
#define I2C2_Base_Address (APB1_Base_Address + 0x00005800)
#define I2C3_Base_Address (APB1_Base_Address + 0x00005C00)
/* CAN1 + CAN2 */
#define CAN1_Base_Address (APB1_Base_Address + 0x00006400)
#define CAN2_Base_Address (APB1_Base_Address + 0x00006800)



/*--------------------------------------------------------------------------------------------------*/
/* Address and define for register in Arm Cortex M4 */
#define NVIC_ISER_Base_Address (0xE000E100)
#define NVIC_ICER_Base_Address (0XE000E180)
#define NVIC_ISPR_Base_Address (0XE000E200)
#define NVIC_ICPR_Base_Address (0XE000E280)
#define NVIC_IABR_Base_Address (0XE000E300)
#define NVIC_IPR_Base_Address  (0XE000E400)

#define System_Timer_Base_Address (0xE000E010)
/* Define register Set-Enable NVIC_ISER */
/* define pointer point to register set,clear interrupt core and priority*/
#define NVIC_IPR ((_vo uint32_t *)(NVIC_IPR_Base_Address))
typedef struct 
{
    _vo uint32_t ISER[8];
} RegDef_NVIC_ISER_t;
#define NVIC_ISER ((RegDef_NVIC_ISER_t *)(NVIC_ISER_Base_Address))

typedef struct 
{
    _vo uint32_t ICER[8];
} RegDef_NVIC_ICER_t;
#define NVIC_ICER ((RegDef_NVIC_ICER_t *)(NVIC_ICER_Base_Address))

/* define pointer point to field system timer,system tick register */
typedef struct 
{
    _vo uint32_t CSR;
    _vo uint32_t RVR;
    _vo uint32_t CVR;
    _vo uint32_t CALIB;

} RegDef_SysTick_t;
#define SysTick ((RegDef_SysTick_t *)(System_Timer_Base_Address))

/*-------------------------------------------------------------------------------------------------------------------------------*/
/* Define register in struct belongs to modules */
/* Flash Inteface Register */
typedef struct 
{
    _vo uint32_t ACR;
    _vo uint32_t KEYR;
    _vo uint32_t OPTKEYR;
    _vo uint32_t SR;
    _vo uint32_t CR;
    _vo uint32_t OPTCR;
} RegDef_FlashInterface_t;
#define FlashInterface ((RegDef_FlashInterface_t *)(FlashInterface_Base_Address))

/* PWR - Power Controller */
typedef struct 
{
    _vo uint32_t CR;
    _vo uint32_t CSR;
} RegDef_PWR_t;
#define PWR ((RegDef_PWR_t *)(PWR_Base_Address))

/* RCC - Reset and clock control register */
typedef struct
{
    _vo uint32_t CR;
    _vo uint32_t PLLCFGR;
    _vo uint32_t CFGR;
    _vo uint32_t CIR;
    _vo uint32_t AHB1RSTR;
    _vo uint32_t AHB2RSTR;
    _vo uint32_t AHB3RSTR;
    _vo uint32_t Reserved_0x1C;
    _vo uint32_t APB1RSTR;
    _vo uint32_t APB2RSTR;
    uint32_t Reserved_0x28_0x2C[2];
    _vo uint32_t AHB1ENR;
    _vo uint32_t AHB2ENR;
    _vo uint32_t AHB3ENR;
    _vo uint32_t Reserved_0x3C;
    _vo uint32_t APB1ENR;
    _vo uint32_t APB2ENR;
    uint32_t Reserved_0x48_0x4C[2];
    _vo uint32_t AHB1LPENR;
    _vo uint32_t AHB2LPENR;
    _vo uint32_t AHB3LPENR;
    _vo uint32_t Reserved_0x5C;
    _vo uint32_t APB1LPENR;
    _vo uint32_t APB2LPENR;
    uint32_t Reserved_0x68_0x6C[2];
    _vo uint32_t BDCR;
    _vo uint32_t CSR;
    uint32_t Reserved_0x78_0x7c[2];
    _vo uint32_t SSCGR;
    _vo uint32_t PLLI2SCFGR;
    _vo uint32_t PLLSAICFGR;
    _vo uint32_t DCKCFGR;

} RegDef_RCC_t;
#define RCC ((RegDef_RCC_t *)(RCC_Base_Address))

/* GPIO - General Peripheral I/O register */
typedef struct
{
    _vo uint32_t MODER;
    _vo uint32_t OTYPER;
    _vo uint32_t OSPEEDR;
    _vo uint32_t PUPDR;
    _vo uint32_t IDR;
    _vo uint32_t ODR;
    _vo uint32_t BSRR;
    _vo uint32_t LCKR;
    _vo uint32_t AFRL;
    _vo uint32_t AFRH;

} RegDef_GPIO_Portx_t;
#define GPIOA ((RegDef_GPIO_Portx_t *)(GPIOA_Base_Address))
#define GPIOB ((RegDef_GPIO_Portx_t *)(GPIOB_Base_Address))
#define GPIOC ((RegDef_GPIO_Portx_t *)(GPIOC_Base_Address))
#define GPIOD ((RegDef_GPIO_Portx_t *)(GPIOD_Base_Address))
#define GPIOE ((RegDef_GPIO_Portx_t *)(GPIOE_Base_Address))
#define GPIOF ((RegDef_GPIO_Portx_t *)(GPIOF_Base_Address))
#define GPIOG ((RegDef_GPIO_Portx_t *)(GPIOG_Base_Address))
#define GPIOH ((RegDef_GPIO_Portx_t *)(GPIOH_Base_Address))
#define GPIOI ((RegDef_GPIO_Portx_t *)(GPIOI_Base_Address))
#define GPIOJ ((RegDef_GPIO_Portx_t *)(GPIOJ_Base_Address))
#define GPIOK ((RegDef_GPIO_Portx_t *)(GPIOK_Base_Address))


/* External interrupt/event - EXTI */
typedef struct 
{
    _vo uint32_t IMR;
    _vo uint32_t EMR;
    _vo uint32_t RTSR;
    _vo uint32_t FTSR;
    _vo uint32_t SWIER;
    _vo uint32_t PR;
} RegDef_EXTI_t;
#define EXTI ((RegDef_EXTI_t *)(EXTI_Base_Address))

/* System configuration controller - SYSCFG*/
typedef struct 
{
    _vo uint32_t MEMRMP;
    _vo uint32_t PMC;
    _vo uint32_t EXTICR[4];
    _vo uint32_t CMPCR;
} RegDef_SYSCFG_t;
#define SYSCFG ((RegDef_SYSCFG_t *)(SYSCFG_Base_Address))

/* Serial peripheral interface (SPI) */
typedef struct 
{
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t CRCPR;
    _vo uint32_t RXCRCR;
    _vo uint32_t TXCRCR;
    _vo uint32_t I2SCFGR;
    _vo uint32_t I2SPR;

} RegDef_SPI_t;
#define SPI1 ((RegDef_SPI_t *)(SPI1_Base_Address))
#define SPI2 ((RegDef_SPI_t *)(SPI2_Base_Address))
#define SPI3 ((RegDef_SPI_t *)(SPI3_Base_Address))
#define SPI4 ((RegDef_SPI_t *)(SPI4_Base_Address))
#define SPI5 ((RegDef_SPI_t *)(SPI5_Base_Address))
#define SPI6 ((RegDef_SPI_t *)(SPI6_Base_Address))

/* Universal synchronous asynchronous receiver 
transmitter (USART)*/
typedef struct 
{
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t BRR;
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t CR3;
    _vo uint32_t GTPR;

} RegDef_USARTx_t;
#define USART1 ((RegDef_USARTx_t *)(USART1_Base_Address))
#define USART2 ((RegDef_USARTx_t *)(USART2_Base_Address))
#define USART3 ((RegDef_USARTx_t *)(USART3_Base_Address))
#define UART4 ((RegDef_USARTx_t *)(UART4_Base_Address))
#define UART5 ((RegDef_USARTx_t *)(UART5_Base_Address))
#define USART6 ((RegDef_USARTx_t *)(USART6_Base_Address))

/* Inter-integrated circuit (I2C) interface */
typedef struct
{
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t OAR1;
    _vo uint32_t OAR2;
    _vo uint32_t DR;
    _vo uint32_t SR1;
    _vo uint32_t SR2;
    _vo uint32_t CCR;
    _vo uint32_t TRISE;
    _vo uint32_t FLTR;

} RegDef_I2Cx_t;
#define I2C1 ((RegDef_I2Cx_t *)(I2C1_Base_Address))
#define I2C2 ((RegDef_I2Cx_t *)(I2C2_Base_Address))
#define I2C3 ((RegDef_I2Cx_t *)(I2C3_Base_Address))


/* Controller area network (bxCAN) */
/* CAN mailbox register */
typedef struct 
{
    _vo uint32_t TIxR;
    _vo uint32_t TDTxR;
    _vo uint32_t TDLxR;
    _vo uint32_t TDHxR;

} RegDef_CANx_Mailbox_t;

/* CAN 2 Receive FIFO register */
typedef struct 
{
    _vo uint32_t RIxR;
    _vo uint32_t RDTxR;
    _vo uint32_t RDLxR;
    _vo uint32_t RDHxR;

} RegDef_CANx_ReceiveFIFO_t;

/* Filter bank i register x (CAN_FiRx) (i=0..27, x=1, 2) */
typedef struct 
{
    _vo uint32_t FiR1;
    _vo uint32_t FiR2;
} RegDef_CANx_FilterBank_t;

/* Can memory map register */
typedef struct 
{   
    /* CAN control and status register */
    _vo uint32_t MCR;
    _vo uint32_t MSR;
    _vo uint32_t TSR;
    _vo uint32_t RF0R;
    _vo uint32_t RF1R;
    _vo uint32_t IER;
    _vo uint32_t ESR;
    _vo uint32_t BTR;
    _vo uint32_t Reserved_0x20_0x17F[88];

    /* CAN mailbox register */
    RegDef_CANx_Mailbox_t CANx_Mailbox[3];

    /* CAN 2 Receive FIFO register */
    RegDef_CANx_ReceiveFIFO_t CANx_ReceiveFIFO[2];

    /* CAN filter register */
    _vo uint32_t Reserved_0x1D0_0x1FF[12];
    _vo uint32_t FMR;
    _vo uint32_t FM1R;
    _vo uint32_t Reserved_0x208;
    _vo uint32_t FS1R;
    _vo uint32_t Reserved_0x210;
    _vo uint32_t FFA1R;
    _vo uint32_t Reserved_0x218;
    _vo uint32_t FA1R;
    _vo uint32_t Reserved_0x220_0x23F[8];
    RegDef_CANx_FilterBank_t CANx_FilterBank[28];

} RegDef_CANx_t;
#define CAN1 ((RegDef_CANx_t *)(CAN1_Base_Address))
#define CAN2 ((RegDef_CANx_t *)(CAN2_Base_Address))


#endif
