#include "stm32f407vg_i2c_driver.h"

/*prototype static func in Func I2C_MasterSendData() */
static void I2C_GenerateStartCondition(RegDef_I2Cx_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(RegDef_I2Cx_t *pI2Cx, uint8_t SlaveAddrPhaseWrite);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CxHandle);
static void I2C_GenerateStopCondition(RegDef_I2Cx_t *pI2Cx);
static void I2C_ManageAcking(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis);
/*static func handle interrupt transmit and receive process */
static void I2C_MasterSendData_Interrupt_Handling(I2C_Handle_t *pI2CxHandle);
static void I2C_MasterReceive_Interrupt_Handling(I2C_Handle_t *pI2CxHandle);
/***************** static func***************/
static void I2C_GenerateStartCondition(RegDef_I2Cx_t *pI2Cx)
{
    /* START: Start generation */
    pI2Cx->CR1 |= (Enable << I2C_CR1_START);
    /* check SB: Start bit (Master mode) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_SB, I2C_Check_Flag_SR1))
        ;
}

static void I2C_ExecuteAddressPhaseWrite(RegDef_I2Cx_t *pI2Cx, uint8_t SlaveAddrPhaseWrite)
{
    /* 7 bit address and 1 bit0 direction write data in position 0  */
    SlaveAddrPhaseWrite = SlaveAddrPhaseWrite << 1;
    SlaveAddrPhaseWrite &= ~(1U << 0);
    pI2Cx->DR = SlaveAddrPhaseWrite;
    /* checking  ADDR: Address sent (master mode)/matched (slave mode) 1: Received address matched.*/
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_ADDR, I2C_Check_Flag_SR1))
        ;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CxHandle)
{
    uint32_t dummy_read;
    /* Check master mode */
    if (I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR2_MSL, I2C_Check_Flag_SR2)) /* Bit 0 MSL: Master/slave   1: Master Mode*/
    {
        /* Disable Ack -*/
        // pI2CxHandle->pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK); /*  0: No acknowledge returned */
        /* Read SR1 and SR2 */
        dummy_read = pI2CxHandle->pI2Cx->SR1;
        dummy_read = pI2CxHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
    else
    {
        /* slave mode,similar read data */
    }
}

static void I2C_GenerateStopCondition(RegDef_I2Cx_t *pI2Cx)
{
    /*  condition to determine stop- wait TxE empty(already tranfer all data) and Bit 2 BTF: Byte transfer finished */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_BTF, I2C_Check_Flag_SR1))
        ; /* 1: Data byte transfer succeeded, clear by hardware after a start or a stop condition*/
    /* generate STOP: Stop generation  1: Stop generation after the current byte transfer or after the current Start condition is sent.*/
    pI2Cx->CR1 |= (Enable << I2C_CR1_STOP);
}

static void I2C_ManageAcking(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pI2Cx->CR1 |= (Enable << I2C_CR1_ACK);
    }
    else
    {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK);
    }
}

static void I2C_ExecuteAddressPhaseRead(RegDef_I2Cx_t *pI2Cx, uint8_t SlaveAddrPhaseRead)
{
    /* 7 bit address and 1 bit1 direction read data in position 0  */
    SlaveAddrPhaseRead = SlaveAddrPhaseRead << 1;
    SlaveAddrPhaseRead |= (1U << 0); /* bit 0 in buffer addr 8bit = 1 is master read data*/
    pI2Cx->DR = SlaveAddrPhaseRead;
    /* checking  ADDR: Address sent (master mode)/matched (slave mode) 1: Received address matched.*/
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_ADDR, I2C_Check_Flag_SR1))
        ;
}

static void I2C_MasterSendData_Interrupt_Handling(I2C_Handle_t *pI2CxHandle)
{
    /* check device is master mode */
    if (I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR2_MSL, I2C_Check_Flag_SR2))
    {
        /*check status of flag TxE -> alway set( empty reg DR ) and Txlen > 0 (have data to tranfer)*/
        if ((I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR1_TxE, I2C_Check_Flag_SR1)) && (pI2CxHandle->Txlen > 0))
        {
            /* similar send data polling */
            pI2CxHandle->pI2Cx->DR = *(pI2CxHandle->TxBuffer); /* assign value into reg */
            pI2CxHandle->Txlen -= 1;                        /* Decrease lenght string tranfer */
            pI2CxHandle->TxBuffer += 1;                     /* point to next element to continue tranfer*/
        }
    }
}

static void I2C_MasterReceiveData_Interrupt_Handling(I2C_Handle_t *pI2CxHandle)
{
    /* Similar Receive data polling */
    /* 2 case (receive 1 byte or >1byte) */
    /*if just send 1 byte -> disable Ack -> slave receive Non-Ack signal to stop transmit data */
    if (pI2CxHandle->Rxlen == 1)
    {
        /* Disable Ack-> after receive data in shift register, master send non-Ack pulse to slave to stop communicate */
        I2C_ManageAcking(pI2CxHandle->pI2Cx, Disable);
        /* read data in DR and take it to Rxbuffer (case : 1byte ) and decrease 1unit Rxlen */
        *(pI2CxHandle->RxBuffer) = pI2CxHandle->pI2Cx->DR;
        pI2CxHandle->Rxlen--;
    }
    else /* Data master receive > 1byte */
    {
        /* receive data and check if it 2nd last data8byte, Disable Ack-> after receive data in shift register (last data),2nd last data in DR ,
        master send non-Ack pulse to slave to stop communicate*/
        if (pI2CxHandle->Rxlen == 2) /* Disable nonAck if 2nd last data already in shift register , master will send nonAck pulse in last data to slave */
        {
            I2C_ManageAcking(pI2CxHandle->pI2Cx, Disable);
        }
        /* read data in DR and take it to Rxbuffer  */
        *(pI2CxHandle->RxBuffer) = pI2CxHandle->pI2Cx->DR;
        pI2CxHandle->RxBuffer += 1;
        pI2CxHandle->Rxlen--;
    }

    /* check lengt data Rxbuffer already receive full data ? */
    if (pI2CxHandle->Rxlen == 0)
    {
        /* Generated stop condition */
        if (pI2CxHandle->CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want to continues tranfer new frame*/
        {
            I2C_GenerateStopCondition(pI2CxHandle->pI2Cx);
        }
        /* already done in receive -> assign value in struct to 0 and Null */
        I2C_CloseReception(pI2CxHandle);
        /*Call applicationevent to notice user already done in interrupt with Event Tx_CMPLT */
        I2C_ApplicationEventCallBack(pI2CxHandle, I2C_EVENT_RX_CMPLT);
    }
}

/*****************************************main func init module I2C*********************************8*/
void I2C_PeripheralClockControl(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_Enable_Clock();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_Enable_Clock();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_Enable_Clock();
        }
    }
    else if (EnOrDis == Disable)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_Disable_Clock();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_Disable_Clock();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_Disable_Clock();
        }
    }
}

void I2C_Initialize(I2C_Handle_t *pI2CxHandle)
{
    /* enable clock in module i2c chanel x */
    I2C_PeripheralClockControl(pI2CxHandle->pI2Cx, Enable);
    /* Declare variable temp to assign each bit field's value (total 15/32bit) in register */
    uint32_t tempreg = 0;
    /* Set ACK in CR1 */
    tempreg |= (pI2CxHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
    /* assign all value in CR1 */
    pI2CxHandle->pI2Cx->CR1 = tempreg;

    /* Set FREQ[5:0] in CR2 */
    tempreg = 0;
    /* In default MCU used main clock is HSI = 16MHz in 2 Bus APB1 APB2 = 16MHz*/
    /* if use funtion SYS_Config_Clock_Tree() to set max clock in each Bus (APB1 = 42MHz) */
    uint32_t DetermineSWS = RCC->CFGR; /*Bits 3:2 SWS: System clock switch status determine main system clock is HSI or PLL */
    uint32_t APBClock = 0;
    if ((DetermineSWS >> 2) & 0x3)
    {
        /* SYS clock is PLL 10: PLL used as the system clock */
        APBClock = 42000000; /* 42Mhz */
    }
    else
    {
        APBClock = 16000000; /*default use HSI clock */
    }
    tempreg |= ((APBClock / 1000000U) << I2C_CR2_FREQ);
    /* assign all value in CR2 */
    pI2CxHandle->pI2Cx->CR2 = (tempreg & 0x3F); /* take 6bit in CR2 FREQ[5:0]: Peripheral clock frequency */

    /* set total address bit in OAR1 - default =  0: 7-bit determine device address */
    tempreg = 0;
    tempreg |= (pI2CxHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7_1);
    tempreg |= (Enable << 14); /*  Bit 14 Should always be kept at 1 by software.*/
    /* assign all value in OAR1 */
    pI2CxHandle->pI2Cx->OAR1 = tempreg;

    /* Set CCR in I2C Clock control register (I2C_CCR) */
    tempreg = 0;
    /* Tlow + Thigh = 1/f(mode) - RM page 870 to determine CCR in 2 mode  */
    uint16_t CCRValue = 0; /*  CCR[11:0]: Clock control register in Fm/Sm mode (Master mode)  */
    if (pI2CxHandle->I2C_Config.I2C_SCLSPEED <= I2C_SCLSPEED_SM)
    {
        /* Standard mode */
        CCRValue = (APBClock) / (2 * (pI2CxHandle->I2C_Config.I2C_SCLSPEED));
        tempreg |= ((CCRValue & 0xFFF) << I2C_CCR_CCR); /* take 12 bit in value and assign into register */
    }
    else
    {
        /* Fast mode */
        tempreg |= (Enable << I2C_CCR_F_S);
        if (pI2CxHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDutyCycle_2)
        {
            /* 0: Fm mode tlow/thigh = 2 */
            CCRValue = (APBClock) / (3 * (pI2CxHandle->I2C_Config.I2C_SCLSPEED));
        }
        else
        {
            /* 1: Fm mode tlow/thigh = 16/9 (see CCR) */
            CCRValue = (APBClock) / (25 * (pI2CxHandle->I2C_Config.I2C_SCLSPEED));
            tempreg |= (Enable << I2C_CCR_DUTY);
        }
        tempreg |= ((CCRValue & 0xFFF) << I2C_CCR_CCR); /* take 12 bit in value and assign into register */
    }
    /* assign all value in I2C Clock control register (I2C_CCR) */
    pI2CxHandle->pI2Cx->CCR = tempreg;

    /* Set TRISE (Time rise in Fm/Sm mode (Master mode)) config available rising edge */
    tempreg = 0;
    /* RM page 871 to determine value TRISE in 2 mode */
    /* TRISE_VAlue = (maximum_Time_rise)/(T_PeripheralClock) + 1 (same unit 1Mhz = 10^6 Hz ,1s = 10^9 ns) */
    if (pI2CxHandle->I2C_Config.I2C_SCLSPEED <= I2C_SCLSPEED_SM)
    {
        /* 0: Sm mode  */
        /* For Standard Mode (Sm), the maximum allowed SCL rise time is typically 1000 ns. */
        tempreg |= ((APBClock / 1000000U) + 1);
    }
    else
    {
        /* 1: Fm mode  */
        /* the Maximum allowed SCL rise time is 300 ns. */
        tempreg |= (((APBClock * 300) / 1000000000U) + 1);
    }
    pI2CxHandle->pI2Cx->TRISE = (tempreg & 0x3F); /* take 6bit and assign into TRISE[5:0]*/
}

void I2C_PeripheralControl(RegDef_I2Cx_t *pI2Cx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pI2Cx->CR1 |= (Enable << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_PE);
    }
}

uint8_t I2C_GetFlagStatus(RegDef_I2Cx_t *pI2Cx, uint32_t FlagName, uint8_t WhatSRReg)
{
    FlagName = (Set << FlagName);
    if (WhatSRReg == I2C_Check_Flag_SR1)
    {
        if (pI2Cx->SR1 & (FlagName))
        {
            return FLAG_ALREADY_SET;
        }
        return FLAG_ALREADY_RESET;
    }
    else if (WhatSRReg == I2C_Check_Flag_SR2)
    {
        if (pI2Cx->SR2 & (FlagName))
        {
            return FLAG_ALREADY_SET;
        }
        return FLAG_ALREADY_RESET;
    }
}

void I2C_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *TxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData)
{
    /* generate START condition and checking SB flag in SR1 */
    I2C_GenerateStartCondition(pI2CxHandle->pI2Cx);
    /* Send Slave Addr and checking  ADDR: Address sent (master mode)/matched (slave mode) in CR1 */
    I2C_ExecuteAddressPhaseWrite(pI2CxHandle->pI2Cx, SlaveAddr);
    /*  EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2. */
    I2C_ClearAddrFlag(pI2CxHandle);

    /* Send data */
    while (Len > 0)
    {
        /* wait TxE empty to write data into DR reg */
        while (!I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR1_TxE, I2C_Check_Flag_SR1))
            ;
        /* write data */
        pI2CxHandle->pI2Cx->DR = *(TxBuffer); /* assign value into reg */
        Len -= 1;                             /* Decrease lenght string tranfer */
        TxBuffer += 1;                        /* point to next element to continue tranfer*/
    }

    /* When tranfer all data -> len = 0 (TxE set )wait flag  Bit 2 BTF: Byte transfer finished in SR1 to qualify generate stop condition */
    while (!I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR1_TxE, I2C_Check_Flag_SR1))
        ;                                                /*  1: Data register empty */
    if (CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want continues tranfer new frame */
    {
        I2C_GenerateStopCondition(pI2CxHandle->pI2Cx);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData)
{
    /* generate START condition and checking SB flag in SR1 */
    I2C_GenerateStartCondition(pI2CxHandle->pI2Cx);
    /* Send Slave Addr and checking  ADDR: Address sent (master mode)/matched (slave mode) in CR1 */
    I2C_ExecuteAddressPhaseRead(pI2CxHandle->pI2Cx, SlaveAddr);

    /*if just send 1 byte -> disable Ack -> slave receive Non-Ack signal to stop transmit data */
    if (Len == 1)
    {
        /* Disable Ack-> after receive data in shift register, master send non-Ack pulse to slave to stop communicate */
        I2C_ManageAcking(pI2CxHandle->pI2Cx, Disable);
        /* Clear Addr flag in SR1 */
        I2C_ClearAddrFlag(pI2CxHandle);
        /* Wait flag Rxne set to determine data already in DR reg and make stop condition */
        while (!I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR1_RxNE, I2C_Check_Flag_SR1))
            ;                                                /*  1: Data register not empty */
        if (CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want continues tranfer new frame */
        {
            I2C_GenerateStopCondition(pI2CxHandle->pI2Cx);
        }
        /* read data in DR and take it to Rxbuffer (case : 1byte ) */
        *RxBuffer = pI2CxHandle->pI2Cx->DR;
    }
    else /* Data master receive > 1byte */
    {
        /* Clear Addr flag in SR1 */
        I2C_ClearAddrFlag(pI2CxHandle);
        /* receive data and check if it 2nd last data8byte, Disable Ack-> after receive data in shift register (last data),2nd last data in DR ,
        master send non-Ack pulse to slave to stop communicate*/
        for (uint32_t i = Len; i > 0; i--)
        {
            if (i == 2) /* Disable nonAck if 2nd last data already in shift register , master will send nonAck pulse in last data to slave */
            {
                I2C_ManageAcking(pI2CxHandle->pI2Cx, Disable);
            }
            /* read data in DR and take it to Rxbuffer  */
            *RxBuffer = pI2CxHandle->pI2Cx->DR;
            RxBuffer += 1;
            /* Wait flag Rxne set to determine data already in DR reg and make stop condition */
            while (!I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_SR1_RxNE, I2C_Check_Flag_SR1))
                ;                                                /*  1: Data register not empty */
            if (CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want continues tranfer new frame */
            {
                I2C_GenerateStopCondition(pI2CxHandle->pI2Cx);
            }
        }
    }
    /* Enable Acking after communicate */
    if (pI2CxHandle->I2C_Config.I2C_AckControl == I2C_AckControl_Enable)
    {
        I2C_ManageAcking(pI2CxHandle->pI2Cx, Enable);
    }
}

uint8_t I2C_MasterSendDataInterruptInit(I2C_Handle_t *pI2CxHandle, uint8_t *TxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData)
{
    uint8_t state = pI2CxHandle->TxRxState;
    if ((state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX)) /* first init doesnt determine TxRxState */
    {
        /* Similar init assign address data like SPI module interrupt ,maybe = 0 ready*/
        pI2CxHandle->TxBuffer = TxBuffer;
        pI2CxHandle->Txlen = Len;
        pI2CxHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CxHandle->CircuitData = CircuitData;
        pI2CxHandle->SlaveAdress = SlaveAddr;
        /* Start Condition */
        I2C_GenerateStartCondition(pI2CxHandle->pI2Cx);
        /* Enable control bit to make start and another sources interrupt
        (Enable bit  ITEVFEN and ITBUFEN in CR2 ) */
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITEVTEN); /* 1: Event interrupt enabled*/
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITBUFEN); /* 1: TxE = 1 or RxNE = 1 generates Event Interrupt (whatever the state of DMAEN)*/
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITERREN); /*  1: Error interrupt enabled*/
        /* pI2CxHandle->pI2Cx->CR2 |= (0x7 << I2C_CR2_ITERREN); /* Enable bit 8-9-10  */
    }
    return state;
}

uint8_t I2C_MasterReceiveDataInterruptInit(I2C_Handle_t *pI2CxHandle, uint8_t *RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t CircuitData)
{
    uint8_t state = pI2CxHandle->TxRxState; /* first init doesnt determine TxRxState, maybe = 0 ready */
    if ((state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX))
    {
        /* Similar init assign address data like SPI module interrupt */
        pI2CxHandle->RxBuffer = RxBuffer;
        pI2CxHandle->Rxlen = Len;
        pI2CxHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CxHandle->CircuitData = CircuitData;
        pI2CxHandle->SlaveAdress = SlaveAddr;
        /* Start Condition */
        I2C_GenerateStartCondition(pI2CxHandle->pI2Cx);
        /* Enable control bit to make start and another sources interrupt
        (Enable bit  ITEVFEN and ITBUFEN in CR2 ) */
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITEVTEN); /* 1: Event interrupt enabled*/
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITBUFEN); /* 1: TxE = 1 or RxNE = 1 generates Event Interrupt (whatever the state of DMAEN)*/
        pI2CxHandle->pI2Cx->CR2 |= (Enable << I2C_CR2_ITERREN); /*  1: Error interrupt enabled*/
        /* pI2CxHandle->pI2Cx->CR2 |= (0x7 << I2C_CR2_ITERREN); /* Enable bit 8-9-10  */
    }
    return state;
}

void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle)
{
    /* Declare 3 temp variable to read bits, determine source interrupt */
    uint32_t temp1 = 0, temp2 = 0, temp3 = 0;
    /* Determine control bit to make event interrupt already enable ? */
    /* Read bit ITEVTEN in CR2 */
    temp1 = pI2CHandle->pI2Cx->CR2 & (Set << I2C_CR2_ITEVTEN);
    /* Read bit ITBUFEN in CR2 */
    temp2 = pI2CHandle->pI2Cx->CR2 & (Set << I2C_CR2_ITBUFEN);
    /* Read flag SB */
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB, I2C_Check_Flag_SR1);

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by Start bit (Flag = SB ,  Enable control bit = ITEVFEN )*/
    if (temp1 && temp3)
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->SlaveAdress);
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->SlaveAdress);
        }
    }

    /*-----------------------------------------------------------------------------------------------------------------*/

    /* Interrupt make by ADDR (Flag = ADDR ,  Enable control bit = ITEVFEN )*/
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR, I2C_Check_Flag_SR1);
    if (temp1 && temp3)
    {
        I2C_ClearAddrFlag(pI2CHandle);
    }

    /* Interrupt make by Data byte transfer finished (flag = BTF , Enable control bit = ITEVFEN)*/
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF, I2C_Check_Flag_SR1);
    if (temp1 && temp3)
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            /* check bit TxE already set (empty reg data -> transmit full data )*/
            if ((I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TxE, I2C_Check_Flag_SR1)) && (pI2CHandle->Txlen == 0))
            {
                /* Generated stop condition */
                if (pI2CHandle->CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want to continues tranfer new frame*/
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
                /* already done in tranfer -> assign value in struct to 0 and Null */
                I2C_CloseTranmission(pI2CHandle);
                /*Call applicationevent to notice user already done in interrupt with Event Tx_CMPLT */
                I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_TX_CMPLT);
            }
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            /* TODO */

            /* check lengt data Rxbuffer already receive full data ? */
            if (pI2CHandle->Rxlen == 0)
            {
                /* Generated stop condition */
                if (pI2CHandle->CircuitData == I2C_ContinuesCircuitData_Disable) /* user don't want to continues tranfer new frame*/
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
                /* already done in receive -> assign value in struct to 0 and Null */
                I2C_CloseReception(pI2CHandle);
                /*Call applicationevent to notice user already done in interrupt with Event Tx_CMPLT */
                I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_RX_CMPLT);
            }
        }
    }

    /*-----------------------------------------------------------------------------------------------------------------*/

    /* Interrupt make by  Stop received (Slave) (flag = STOPF , Enable control bit = ITEVFEN)*/
    /*  Set by hardware when a Stop condition is detected on the bus by the slave after an
acknowledge (if ACK=1)*/
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_STOPF, I2C_Check_Flag_SR1);
    if (temp1 && temp3)
    {
        /*  Cleared by software reading the SR1 register followed by a write in the CR1 register, */
        pI2CHandle->pI2Cx->CR1 |= 0x0000; /* clear STOPF */
        /*Call applicationevent to notice user already done in interrupt with Event STOPF */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_STOPF);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/

    /*****Interrupt make by   Transmit buffer empty (flag = TxE , Enable control bit = ITEVFEN + ITBUFEN)***/
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TxE, I2C_Check_Flag_SR1);
    if (temp1 && temp2 && temp3)
    {
        /* call static func to handle tranfer process */
        I2C_MasterSendData_Interrupt_Handling(pI2CHandle);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/

    /*****Interrupt make by  Receive buffer not empty (flag = RxNE , Enable control bit = ITEVFEN + ITBUFEN)***/
    temp3 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RxNE, I2C_Check_Flag_SR1);
    if (temp1 && temp2 && temp3)
    {
        /* call static func to handle reception process */
        I2C_MasterReceiveData_Interrupt_Handling(pI2CHandle);
    }
}

void I2C_CloseTranmission(I2C_Handle_t *pI2CxHandle)
{
    /* Disable control bit make interrupt */
    pI2CxHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITEVTEN); /* 0: Event interrupt Disabled*/
    pI2CxHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITBUFEN); /* 0: TxE = 1 or RxNE = 1 not generates Event Interrupt (whatever the state of DMAEN)*/
    /* assign value in struct back to new */
    pI2CxHandle->TxBuffer = NULL;
    pI2CxHandle->Txlen = 0;
    pI2CxHandle->TxRxState = I2C_READY;
}

void I2C_CloseReception(I2C_Handle_t *pI2CxHandle)
{
    /* Disable control bit make interrupt */
    pI2CxHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITEVTEN); /* 0: Event interrupt Disabled*/
    pI2CxHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITBUFEN); /* 0: TxE = 1 or RxNE = 1 not generates Event Interrupt (whatever the state of DMAEN)*/
    /* assign value in struct back to new */
    pI2CxHandle->RxBuffer = NULL;
    pI2CxHandle->Rxlen = 0;
    pI2CxHandle->TxRxState = I2C_READY;

    /* Enable Acking after communicate */
    if (pI2CxHandle->I2C_Config.I2C_AckControl == I2C_AckControl_Enable)
    {
        I2C_ManageAcking(pI2CxHandle->pI2Cx, Enable);
    }
}

void I2C_IRQErrorHandling(I2C_Handle_t *pI2CHandle)
{
    /* Declare 3 temp variable to read bits, determine source interrupt in error */
    uint32_t temp1 = 0, temp2 = 0;
    /* Determine control bit to make error interrupt already enable ? */
    /* Read bit  ITERREN in CR2 */
    temp1 = pI2CHandle->pI2Cx->CR2 & (Set << I2C_CR2_ITERREN);

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by Bus error (Flag =  BERR ,  Enable control bit = ITERREN )*/
    temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BERR, I2C_Check_Flag_SR1);
    if (temp1 && temp2)
    {
        /*  Set by hardware when the interface detects an SDA rising or falling edge while SCL is high,
occurring in a non-valid position during a byte transfer.*/
        /*  Cleared by software writing 0, or by hardware when PE=0 */
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_BERR);
        /*Call applicationevent to notice user already done in interrupt with I2C_EVENT_BERR */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_BERR);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by  ARLO: Arbitration lost (master mode)  (Flag =  ARLO ,  Enable control bit = ITERREN )*/
    temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ARLO, I2C_Check_Flag_SR1);
    if (temp1 && temp2)
    {
        /* Set by hardware when the interface loses the arbitration of the bus to another master*/
        /*  Cleared by software writing 0, or by hardware when PE=0 */
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_ARLO);
        /*Call applicationevent to notice user already done in interrupt with I2C_EVENT_ARLO */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_ARLO);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by  AF:  AF: Acknowledge failure  (Flag =  AF ,  Enable control bit = ITERREN )*/
    temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_AF, I2C_Check_Flag_SR1);
    if (temp1 && temp2)
    {
        /* Set by hardware when no acknowledge is returned.*/
        /*  Cleared by software writing 0, or by hardware when PE=0 */
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_AF);
        /*Call applicationevent to notice user already done in interrupt with I2C_EVENT_ARLO */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_AF);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by  OVR:   OVR: Overrun/Underrun  (Flag =  OVR ,  Enable control bit = ITERREN )*/
    temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_OVR, I2C_Check_Flag_SR1);
    if (temp1 && temp2)
    {
        /* Set by hardware in slave mode when NOSTRETCH=1 and:– In reception when a new byte is received (including ACK pulse) and the DR register has not
been read yet. New received byte is lost.– In transmission when a new byte should be sent and the DR register has not been written
yet. The same byte is sent twice.*/
        /*  Cleared by software writing 0, or by hardware when PE=0 */
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_OVR);
        /*Call applicationevent to notice user already done in interrupt with I2C_EVENT_ARLO */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_OVR);
    }

    /*-----------------------------------------------------------------------------------------------------------------*/
    /* Interrupt make by  TIMEOUT: Timeout or Tlow error:   (Flag =  TIMEOUT ,  Enable control bit = ITERREN )*/
    temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TIMEOUT, I2C_Check_Flag_SR1);
    if (temp1 && temp2)
    {
        /*  RM page 866 */
        /*  Cleared by software writing 0, or by hardware when PE=0 */
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_TIMEOUT);
        /*Call applicationevent to notice user already done in interrupt with I2C_EVENT_ARLO */
        I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_TIMEOUT);
    }
}

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CxHandle, uint8_t ApEvent)
{
}
