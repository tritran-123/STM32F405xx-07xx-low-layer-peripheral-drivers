#ifndef STM32F407VG_UART_H
#define STM32F407VG_UART_H

#include "stm32f407vg.h"

/* Function enable + disable clock module usart/uart for each Peripherals */
/* bit in register (RCC_APB1ENR) */
#define USART2EN 17
#define USART3EN 18
#define UART4EN 19
#define UART5EN 20
/* bit in register (RCC_APB2ENR) */
#define USART1EN 4
#define USART6EN 5
/* Enable clock func */
#define USART2_Enable_Clock() ((RCC-> APB1ENR) |= (1 << USART2EN))
#define USART3_Enable_Clock() ((RCC-> APB1ENR) |= (1 << USART3EN))
#define UART4_Enable_Clock() ((RCC-> APB1ENR) |= (1 << UART4EN))
#define UART5_Enable_Clock() ((RCC-> APB1ENR) |= (1 << UART5EN))
#define USART1_Enable_Clock() ((RCC-> APB2ENR) |= (1 << USART1EN))
#define USART6_Enable_Clock() ((RCC-> APB2ENR) |= (1 << USART6EN))
/* Disable clock func */
#define USART2_Disable_Clock() ((RCC-> APB1ENR) &= ~(1 << USART2EN))
#define USART3_Disable_Clock() ((RCC-> APB1ENR) &= ~(1 << USART3EN))
#define UART4_Disable_Clock() ((RCC-> APB1ENR) &= ~(1 << UART4EN))
#define UART5_Disable_Clock() ((RCC-> APB1ENR) &= ~(1 << UART5EN))
#define USART1_Disable_Clock() ((RCC-> APB2ENR) &= ~(1 << USART1EN))
#define USART6_Disable_Clock() ((RCC-> APB2ENR) &= ~(1 << USART6EN))

/* define each location bit field in register */
/* Status register (USART_SR) */
#define USART_SR_PE     0
#define USART_SR_FE     1
#define USART_SR_NF     2
#define USART_SR_ORE    3
#define USART_SR_IDLE   4
#define USART_SR_RXNE   5
#define USART_FLAG_RXNE (1 << USART_SR_RXNE)
#define USART_SR_TC     6
#define USART_FLAG_TC   (1 << USART_SR_TC)
#define USART_SR_TXE    7
#define USART_FLAG_TXE  (1 << USART_SR_TXE)
#define USART_SR_LBD    8
#define USART_SR_CTS    9
/*Baud rate register (USART_BRR)*/
#define USART_BRR_DIV_Fraction  0
#define USART_BRR_DIV_Mantissa  4
/* Control register 1 (USART_CR1) */
#define USART_CR1_SBK       0
#define USART_CR1_RWU       1
#define USART_CR1_RE        2
#define USART_CR1_TE        3
#define USART_CR1_IDLEIE    4
#define USART_CR1_RXNEIE    5
#define USART_CR1_TCIE      6
#define USART_CR1_TXEIE     7
#define USART_CR1_PEIE      8
#define USART_CR1_PS        9
#define USART_CR1_PCE       10
#define USART_CR1_WAKE      11
#define USART_CR1_M         12
#define USART_CR1_UE        13
#define USART_CR1_OVER8     15
/* Control register 2 (USART_CR2) */
#define USART_CR2_ADD       0
#define USART_CR2_LBDL      5
#define USART_CR2_LBDIE     6
#define USART_CR2_LBCL      8
#define USART_CR2_CPHA      9
#define USART_CR2_CPOL      10
#define USART_CR2_CLKEN     11
#define USART_CR2_STOP      12
#define USART_CR2_LINEN     14
/* Control register 3 (USART_CR3) */
#define USART_CR3_CTSE      9
#define USART_CR3_RTSE      8
#define USART_CR3_EIE       0
/* define information about status each bit field */
/* Mode */
#define USART_Mode_Only_TX  0
#define USART_Mode_Only_RX  1
#define USART_Mode_TXRX     2
/* BaudRate */
#define USART_BaudRate_1200     1200
#define USART_BaudRate_2400     2400
#define USART_BaudRate_9600     9600
#define USART_BaudRate_19200    19200
#define USART_BaudRate_38400    38400
#define USART_BaudRate_57600    57600
#define USART_BaudRate_115200   115200
#define USART_BaudRate_230400   230400
#define USART_BaudRate_460800   460800
#define USART_BaudRate_921600   921600
#define USART_BaudRate_2M       2000000
#define USART_BaudRate_3M       3000000
/* NoOfStopBits */
#define USART_NoOfStopBits_1StopBits    0
#define USART_NoOfStopBits_0_5StopBits  1
#define USART_NoOfStopBits_2StopBits    2
#define USART_NoOfStopBits_1_5StopBits  3
/* WordLenght */
#define USART_WordLenght_8DataBits  0
#define USART_WordLenght_9DataBits  1
/* ParityControl */
#define USART_ParityControl_EN_ODD   0
#define USART_ParityControl_EN_EVEN  1
#define USART_ParityControl_DIS      2
/* HWFlowControl */
#define USART_HWFlowControl_NONE     0
#define USART_HWFlowControl_CTS      1
#define USART_HWFlowControl_RTS      2
#define USART_HWFlowControl_CTS_RTS  3
/* USART Flag for interrupt */
#define USART_READY 0
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
/* USART FLAG EVENT INTERRUPT TO CALL APPLICATION */
#define USART_EVENT_TX_CMPLT 0
#define USART_EVENT_RX_CMPLT 1
#define USART_EVENT_CTS      2
#define USART_EVENT_IDLE     3
#define USART_EVENT_PE       4
#define USART_EVENT_LBD      5
#define USART_EVENT_NF       6
#define USART_EVENT_FE       7
#define USART_EVENT_ORE      8



typedef struct 
{
    uint8_t USART_Mode;
    uint32_t USART_BaudRate;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLenght;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;

} USART_Config_t;

typedef struct 
{
    RegDef_USARTx_t *pUSARTx;
    USART_Config_t USART_Config;
    uint8_t *TxBuffer;
    uint8_t *RxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;

} USART_Handle_t;

/* prototype func */
void USART_PeripheralClockControl(RegDef_USARTx_t *pUSARTx, uint8_t EnOrDis);
void USART_Initialize(USART_Handle_t *pUSARTxHandle);
void USART_SetBaudRate(RegDef_USARTx_t *pUSARTx,uint32_t BaudRate);
void USART_PeripheralControl(RegDef_USARTx_t *pUSARTx,uint8_t EnOrDis);
uint8_t USART_GetFlagStatus(RegDef_USARTx_t *pUSARTx, uint32_t FlagName);
void USART_SendData(USART_Handle_t *pUSARTxHandle, uint8_t *TxBuffer, uint32_t Len);
void USART_SendChar(RegDef_USARTx_t *pUSARTx, uint8_t TxBuffer);
void USART_ReceiveData(USART_Handle_t *pUSARTxHandle, uint8_t *RxBuffer, uint32_t Len);
/* Interrupt func */
uint8_t USART_SendDataInterruptInit(USART_Handle_t *pUSARTxHandle, uint8_t *TxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataInterruptInit(USART_Handle_t *pUSARTxHandle, uint8_t *RxBuffer, uint32_t Len);
void USART_IRQHandling(USART_Handle_t *pUSARTxHandle);
void USART_TC_Interrupt_Handle(USART_Handle_t *pUSARTxHandle);
void USART_TXE_Interrupt_Handle(USART_Handle_t *pUSARTxHandle);
void USART_RXNE_Interrupt_Handle(USART_Handle_t *pUSARTxHandle);

void USART_ApplicationEventCallBack(USART_Handle_t *pUSARTxHandle, uint8_t event);

#endif