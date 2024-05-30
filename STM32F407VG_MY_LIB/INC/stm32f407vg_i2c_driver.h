#ifndef STM32F407VG_I2C_H
#define STM32F407VG_I2C_H

#include "stm32f407vg.h"

/* Function enable + disable clock module i2c for each Peripherals */
/* bit in register (RCC_APB1ENR) */
#define I2C1EN 21
#define I2C2EN 22
#define I2C3EN 23
/* Enable clock func */
#define I2C1_Enable_Clock() ((RCC->APB1ENR) |= (Enable << I2C1EN))
#define I2C2_Enable_Clock() ((RCC->APB1ENR) |= (Enable << I2C2EN))
#define I2C3_Enable_Clock() ((RCC->APB1ENR) |= (Enable << I2C3EN))
/* Disable clock func */
#define I2C1_Disable_Clock() ((RCC->APB1ENR) |= (Enable << I2C1EN))
#define I2C2_Disable_Clock() ((RCC->APB1ENR) |= (Enable << I2C2EN))
#define I2C3_Disable_Clock() ((RCC->APB1ENR) |= (Enable << I2C3EN))

/* define each location bit field in register */
/* I2C Control register 1 (I2C_CR1) */
#define I2C_CR1_PE 0
#define I2C_CR1_SMBUS 1
#define I2C_CR1_SMBTYPE 3
#define I2C_CR1_ENARP 4
#define I2C_CR1_ENPEC 5
#define I2C_CR1_ENGC 6
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1_POS 11
#define I2C_CR1_PEC 12
#define I2C_CR1_ALERT 13
#define I2C_CR1_SWRST 15
/* I2C Control register 2 (I2C_CR2) */
#define I2C_CR2_FREQ 0 /* BIT [0:5] */
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10
#define I2C_CR2_DMAEN 11
#define I2C_CR2_LAST 12
/* I2C Own address register 1 (I2C_OAR1) */
#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD7_1 1  /* BIT [1:7] */
#define I2C_OAR1_ADD09_8 8 /* BIT [8:9] */
#define I2C_OAR1_ADDMODE 15
/* I2C Own address register 2 (I2C_OAR2) */
#define I2C_OAR2_ENDUAL 0
#define I2C_OAR2_ADD2 1 /* BIT [1:7] */
/* I2C Data register (I2C_DR) BIT[0:7] */
/* I2C Status register 1 (I2C_SR1) */
#define I2C_Check_Flag_SR1 1 /* Serves for I2C_GetFlagStatus() func */
#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RxNE 6
#define I2C_SR1_TxE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARLO 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_PECERR 12
#define I2C_SR1_TIMEOUT 14
#define I2C_SR1_SMBALERT 15
/* I2C Status register 2 (I2C_SR2) */
#define I2C_Check_Flag_SR2 2 /* Serves for I2C_GetFlagStatus() func */
#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_SMBDEFAULT 5
#define I2C_SR2_SMBHOST 6
#define I2C_SR2_DUALF 7
#define I2C_SR2_PEC0_7 8 /* BIT [8:15] */
/* I2C Clock control register (I2C_CCR) */
#define I2C_CCR_CCR 0 /* BIT [0:11] */
#define I2C_CCR_DUTY 14
#define I2C_CCR_F_S 15
/* define information about status each bit field */
/* I2C SCLSPEED */
#define I2C_SCLSPEED_SM 100000 /*  the standard mode (Sm, up to 100 kHz)*/
#define I2C_SCLSPEED_FM 400000 /* Fm mode (Fm, up to 400 kHz).  */
/* I2C ACKControl */
#define I2C_AckControl_Enable 1
#define I2C_AckControl_Disable 0
/* I2C_FMDutyCycle bit 14 in I2C Clock control register (I2C_CCR)*/
#define I2C_FMDutyCycle_2 0    /* 0: Fm mode tlow/thigh = 2 */
#define I2C_FMDutyCycle_16_9 1 /* 1: Fm mode tlow/thigh = 16/9 (see CCR)*/
/* I2C_CircuitData */
/* this define determine when data already transmit or receiver, user want to continues communicate (newframe -startbit +......) or make stop condition */
#define I2C_ContinuesCircuitData_Enable 1
#define I2C_ContinuesCircuitData_Disable 0
/* I2C Flag for interrupt */
#define I2C_READY 0
#define I2C_BUSY_IN_TX 1
#define I2C_BUSY_IN_RX 2
/* I2C FLAG EVENT INTERRUPT TO CALL APPLICATION */
#define I2C_EVENT_TX_CMPLT 0
#define I2C_EVENT_RX_CMPLT 1
#define I2C_EVENT_STOPF 2
#define I2C_EVENT_BERR 3
#define I2C_EVENT_ARLO 4
#define I2C_EVENT_AF 5
#define I2C_EVENT_OVR 6
#define I2C_EVENT_TIMEOUT 7


typedef struct
{
    uint32_t I2C_SCLSPEED;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_AckControl;
    uint8_t I2C_FMDutyCycle;

} I2C_Config_t;

typedef struct
{
    RegDef_I2Cx_t *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t *TxBuffer;
    uint8_t *RxBuffer;
    uint32_t Txlen;
    uint32_t Rxlen;
    uint8_t TxRxState;   /* 2 buffer transmit and receive in one 1 line SDA -> just 1 flag to determine state */
    uint8_t CircuitData; /* determine user want to continues tranfer new frame or creat stop condition */
    uint8_t SlaveAdress; /* use in I2C_IRQEventHandling */

} I2C_Handle_t;

/* prototype func */
void I2C_PeripheralClockControl(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis);
void I2C_Initialize(I2C_Handle_t *pI2CxHandle);
void I2C_PeripheralControl(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis);
uint8_t I2C_GetFlagStatus(RegDef_I2Cx_t *pI2Cx, uint32_t FlagName, uint8_t WhatSRReg);
void I2C_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *TxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData);    /* Basic func tranfer data, just tranfer 1 address(master) in 1 time - non use ENDUAL: Dual addressing mode enable*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData); /* Basic func receive data, just receive 1 address(master) in 1 time - non use ENDUAL: Dual addressing mode enable*/
/* interrupt func */
uint8_t I2C_MasterSendDataInterruptInit(I2C_Handle_t *pI2CxHandle, uint8_t *TxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData);
uint8_t I2C_MasterReceiveDataInterruptInit(I2C_Handle_t *pI2CxHandle, uint8_t *RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData);
void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseTranmission(I2C_Handle_t *pI2CxHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CxHandle);
void I2C_IRQErrorHandling(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CxHandle, uint8_t ApEvent);

#endif
