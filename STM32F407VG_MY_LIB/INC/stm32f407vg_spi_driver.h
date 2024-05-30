
#ifndef STM32F407VG_SPI_H
#define STM32F407VG_SPI_H

#include "stm32f407vg.h"

/* Function enable clock module spi for each chanel */
/* bit in register (RCC_APB1ENR) */
#define SPI2EN 14
#define SPI3EN 15
/* bit in register (RCC_APB2ENR) */
#define SPI1EN 12

#define SPI2_Enable_Clock() ((RCC->APB1ENR) |= (1 << SPI2EN))
#define SPI3_Enable_Clock() ((RCC->APB1ENR) |= (1 << SPI3EN))
#define SPI1_Enable_Clock() ((RCC->APB2ENR) |= (1 << SPI1EN))

#define SPI2_Disable_Clock() ((RCC->APB1ENR) &= ~(1U << SPI2EN))
#define SPI3_Disable_Clock() ((RCC->APB1ENR) &= ~(1U << SPI3EN))
#define SPI1_Disable_Clock() ((RCC->APB2ENR) &= ~(1U << SPI1EN))

/* define information about status each bit field */
#define SPI_DeviceMode_Master 1
#define SPI_DeviceMode_Slave 0

#define SPI_BusConfig_FullDuplex 0
#define SPI_BusConfig_HalfDuplex_RXONLY 0
#define SPI_BusConfig_HalfDuplex_TXONLY 1
#define SPI_BusConfig_RXOnly 1

#define SPI_SclkSpeed_Div2 0
#define SPI_SclkSpeed_Div4 1
#define SPI_SclkSpeed_Div8 2
#define SPI_SclkSpeed_Div16 3
#define SPI_SclkSpeed_Div32 4
#define SPI_SclkSpeed_Div64 5
#define SPI_SclkSpeed_Div128 6
#define SPI_SclkSpeed_Div256 7

#define SPI_DFF_8bit 0
#define SPI_DFF_16bit 1

#define SPI_CPOL_LowIdle 0
#define SPI_CPOL_HighIdle 1
#define SPI_CPHA_FirstClock 0
#define SPI_CPHA_SecondClock 1

#define SPI_SSM_EN 1
#define SPI_SSM_DIS 0

/* define each location bit field in register */
/* SPI control register 1 (SPI_CR1) */
#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR 3
#define SPI_CR1_SPE 6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSI 8
#define SPI_CR1_SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_DFF 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15
/* SPI control register 2 (SPI_CR2) */
#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIE 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXEIE 7
/*  SPI status register (SPI_SR) */
#define SPI_SR_RXNE 0
#define SPI_FLAG_RXNE (1 << SPI_SR_RXNE)
#define SPI_SR_TXE 1
#define SPI_FLAG_TXE (1 << SPI_SR_TXE)
#define SPI_SR_CHSIDE 2
#define SPI_SR_UDR 3
#define SPI_SR_CRCERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BSY 7
#define SPI_FLAG_BSY (1 << SPI_SR_BSY)
#define SPI_SR_FRE 8
/* SPI Flag for interrupt */
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/* define struct to determine data about spi */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF; /* Data frame format (8 or 16bit data )*/
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;

} SPI_Config_t;

typedef struct
{
    RegDef_SPI_t *pSPIx;
    SPI_Config_t SPI_Config;
    /* variable for interrupt module*/
    uint8_t *TxBuffer; /* pointer point to first element in data want to send */
    uint8_t *RxBuffer; /* pointer point to first element in data want to receive */
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;

} SPI_Handle_t;

/*prototype func */
void SPI_PeripheralClockControl(RegDef_SPI_t *pSPIx, uint8_t EnOrDis);
void SPI_Initialize(SPI_Handle_t *pSPIxHandle);
uint8_t SPI_GetFlagStatus(RegDef_SPI_t *pSPIx, uint32_t FlagName);
void SPI_SendData(RegDef_SPI_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(RegDef_SPI_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_PeripheralEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis);
void SPI_SSIEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis);
void SPI_SSOEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis);

/* Function service for interrupt module */
uint8_t SPI_SendDataInterruptInit(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataInterruptInit(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle);
void SPI_CloseTranmission(SPI_Handle_t *pSPIxHandle);
void SPI_CloseReceive(SPI_Handle_t *pSPIxHandle);
void SPI_Send16bit(RegDef_SPI_t *pSPIx, uint16_t pTxBuffer);

#endif
