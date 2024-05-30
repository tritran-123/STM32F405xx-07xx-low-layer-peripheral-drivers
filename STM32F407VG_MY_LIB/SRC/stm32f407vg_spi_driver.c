#include "stm32f407vg_spi_driver.h"
/*Prototype static function */
static void SPI_TX_Interrupt_Handle(SPI_Handle_t *pSPIxHandle);
static void SPI_RX_Interrupt_Handle(SPI_Handle_t *pSPIxHandle);
static void SPI_OVR_Interrupt_Handle(SPI_Handle_t *pSPIxHandle);

void SPI_PeripheralClockControl(RegDef_SPI_t *pSPIx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        if (pSPIx == SPI1)
        {
            SPI1_Enable_Clock();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_Enable_Clock();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_Enable_Clock();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_Disable_Clock();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_Disable_Clock();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_Disable_Clock();
        }
    }
}

void SPI_Initialize(SPI_Handle_t *pSPIxHandle)
{
    /* enable clock in module spi chanel x */
    SPI_PeripheralClockControl(pSPIxHandle->pSPIx, Enable);
    /* Declare variable temp to assign each bit field's value (total 15/32bit) in register SPI control register 1 (SPI_CR1) */
    uint32_t tempreg = 0;
    /* Set device mode*/
    tempreg |= (pSPIxHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);
    /* Set Busconfig */
    if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_BusConfig_FullDuplex)
    {
        tempreg &= ~(1U << SPI_CR1_BIDIMODE);
    }
    else if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_BusConfig_HalfDuplex_RXONLY)
    {
        tempreg |= (1 << SPI_CR1_BIDIMODE);
        tempreg &= ~(1U << SPI_CR1_BIDIOE);
    }
    else if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_BusConfig_HalfDuplex_TXONLY)
    {
        tempreg |= (1 << SPI_CR1_BIDIMODE);
        tempreg |= (SPI_BusConfig_HalfDuplex_TXONLY << SPI_CR1_BIDIOE);
    }
    else if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_BusConfig_RXOnly)
    {
        tempreg &= ~(1U << SPI_CR1_BIDIMODE);
        tempreg |= (SPI_BusConfig_RXOnly << SPI_CR1_BIDIOE);
    }
    /* Set SclkSpeed - HSI = 16MHz*/
    tempreg |= ((uint32_t)pSPIxHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);
    /* Set DFF 8 or 16 bit data */
    tempreg |= ((uint32_t)pSPIxHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);
    /* Set CPOL */
    tempreg |= ((uint32_t)pSPIxHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);
    /* Set CPHA */
    tempreg |= ((uint32_t)pSPIxHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);
    /* Set SSM */
    tempreg |= ((uint32_t)pSPIxHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

    /* LSB */
    /*tempreg |= (1 << 7);*/

    /* assign all value in CR1 */
    pSPIxHandle->pSPIx->CR1 = tempreg;
}

uint8_t SPI_GetFlagStatus(RegDef_SPI_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & (FlagName))
    {
        return FLAG_ALREADY_SET;
    }
    return FLAG_ALREADY_RESET;
}

void SPI_SendData(RegDef_SPI_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        /* enable module spi */
        SPI_PeripheralEnable(pSPIx, Enable);
        /* wait buffer empty*/
        while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_ALREADY_RESET)
            ;
        /* Check Data sending in 1 time is 8 or 16 bit */
        if (pSPIx->CR1 & (SPI_DFF_16bit << SPI_CR1_DFF))
        {
            /* if buffer send = 16bit */
            pSPIx->DR = (*((uint16_t *)pTxBuffer)); /* pointer cash in 16bit it will tranfer 16bit = 2 lement 8 bit in 1 time*/
            Len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            /* if buffer send = 8bit */
            pSPIx->DR = *(pTxBuffer);
            Len--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(RegDef_SPI_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        /* wait buffer full data*/
        while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_ALREADY_RESET)
            ;
        /* Check Data sending in 1 time is 8 or 16 bit */
        if (pSPIx->CR1 & (SPI_DFF_16bit << SPI_CR1_DFF))
        {
            /* if buffer send = 16bit */
            (*((uint16_t *)pRxBuffer)) = pSPIx->DR; /* pointer cash in 16bit it will tranfer 16bit = 2 lement 8 bit in 1 time*/
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            /* if buffer send = 8bit */
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

void SPI_PeripheralEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pSPIx->CR1 |= (Enable << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
    }
}

void SPI_SSIEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pSPIx->CR1 |= (Enable << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
    }
}

void SPI_SSOEnable(RegDef_SPI_t *pSPIx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pSPIx->CR2 |= (Enable << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR2_SSOE);
    }
}

/* iNTERRUPT FUNC */
uint8_t SPI_SendDataInterruptInit(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIxHandle->TxState;
    if (state != SPI_BUSY_IN_TX)
    {
        /*1. Save the address Txbuffer and len into struct */
        pSPIxHandle->TxBuffer = pTxBuffer;
        pSPIxHandle->TxLen = Len;
        /*2. Change the SPI_State as busy in tranmission */
        pSPIxHandle->TxState = SPI_BUSY_IN_TX;
        /*3. Enable the TXEIE: Tx buffer empty interrupt enable */
        pSPIxHandle->pSPIx->CR2 |= (Enable << SPI_CR2_TXEIE);
    }
    return state;
}

uint8_t SPI_ReceiveDataInterruptInit(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIxHandle->RxState;
    if (state != SPI_BUSY_IN_RX)
    {
        /*1. Save the address Txbuffer and len into struct */
        pSPIxHandle->RxBuffer = pRxBuffer;
        pSPIxHandle->RxLen = Len;
        /*2. Change the SPI_State as busy in tranmission */
        pSPIxHandle->RxState = SPI_BUSY_IN_RX;
        /*3. Enable the TXEIE: Tx buffer empty interrupt enable */
        pSPIxHandle->pSPIx->CR2 |= (Enable << SPI_CR2_RXNEIE);
    }
    return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle)
{
    /*1. Read state in flag TXE (set when buffer empty->ready to transmission ) and bit TXEIE already have set ?*/
    /* TXE */
    uint16_t temp1, temp2;
    temp1 = pSPIxHandle->pSPIx->CR2 & (Set << SPI_CR2_TXEIE);
    temp2 = pSPIxHandle->pSPIx->SR & (Set << SPI_SR_TXE);
    /* Check 2 bit txe and txeie already set and handle send data */
    if (temp1 && temp2)
    {
        /* handle TXE */
        SPI_TX_Interrupt_Handle(pSPIxHandle);
    }

    /*2. Similar */
    /* RXNE */
    temp1 = 0;
    temp2 = 0;
    temp1 = pSPIxHandle->pSPIx->CR2 & (Set << SPI_CR2_RXNEIE);
    temp2 = pSPIxHandle->pSPIx->SR & (Set << SPI_SR_RXNE);
    /* Check 2 bit txe and txeie already set and handle receive data */
    if (temp1 && temp2)
    {
        /* handle RXNE */
        SPI_RX_Interrupt_Handle(pSPIxHandle);
    }

    /* 3. Hadle Error by ovr flag  */
    temp1 = 0;
    temp2 = 0;
    temp1 = pSPIxHandle->pSPIx->CR2 & (Set << SPI_CR2_ERRIE);
    temp2 = pSPIxHandle->pSPIx->SR & (Set << SPI_SR_OVR);
    /* Check 2 bit txe and txeie already set and handle receive data */
    if (temp1 && temp2)
    {
        /* handle RXNE */
        SPI_OVR_Interrupt_Handle(pSPIxHandle);
    }
}

static void SPI_TX_Interrupt_Handle(SPI_Handle_t *pSPIxHandle)
{
    /* Check Data sending in 1 time is 8 or 16 bit */
    if (pSPIxHandle->pSPIx->CR1 & (SPI_DFF_16bit << SPI_CR1_DFF))
    {
        /* if buffer send = 16bit */
        pSPIxHandle->pSPIx->DR = (*((uint16_t *)pSPIxHandle->TxBuffer)); /* pointer cash in 16bit it will tranfer 16bit = 2 lement 8 bit in 1 time*/
        pSPIxHandle->TxLen -= 2;
        pSPIxHandle->TxBuffer += 2;
    }
    else
    {
        /* if buffer send = 8bit */
        pSPIxHandle->pSPIx->DR = *(pSPIxHandle->TxBuffer);
        pSPIxHandle->TxLen--;
        pSPIxHandle->TxBuffer++;
    }

    /* close tranfer when len = 0 */
    if (!pSPIxHandle->TxLen)
    {
        SPI_CloseTranmission(pSPIxHandle);
    }
}

void SPI_CloseTranmission(SPI_Handle_t *pSPIxHandle)
{
    pSPIxHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_TXEIE);
    pSPIxHandle->TxBuffer = NULL;
    pSPIxHandle->TxLen = 0;
    pSPIxHandle->TxState = SPI_READY;
}

static void SPI_RX_Interrupt_Handle(SPI_Handle_t *pSPIxHandle)
{
    /* Check Data sending in 1 time is 8 or 16 bit */
    if (pSPIxHandle->pSPIx->CR1 & (SPI_DFF_16bit << SPI_CR1_DFF))
    {
        /* if buffer send = 16bit */
        (*((uint16_t *)pSPIxHandle->RxBuffer)) = pSPIxHandle->pSPIx->DR; /* pointer cash in 16bit it will tranfer 16bit = 2 lement 8 bit in 1 time*/
        pSPIxHandle->RxLen -= 2;
        pSPIxHandle->RxBuffer += 2;
    }
    else
    {
        /* if buffer send = 8bit */
        *(pSPIxHandle->RxBuffer) = pSPIxHandle->pSPIx->DR;
        pSPIxHandle->RxLen--;
        pSPIxHandle->RxBuffer++;
    }

    /* close receive when len = 0 */
    if (!pSPIxHandle->RxLen)
    {
        SPI_CloseReceive(pSPIxHandle);
    }
}

void SPI_CloseReceive(SPI_Handle_t *pSPIxHandle)
{
    pSPIxHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_RXNEIE);
    pSPIxHandle->RxBuffer = NULL;
    pSPIxHandle->RxState = SPI_READY;
    pSPIxHandle->RxLen = 0;
}

static void SPI_OVR_Interrupt_Handle(SPI_Handle_t *pSPIxHandle)
{
    uint16_t temp;
    (void)temp; /* speeze type to void to avoid warning */
    /* clear the overun flag*/
    temp = pSPIxHandle->pSPIx->SR;
    temp = pSPIxHandle->pSPIx->DR;
}


void SPI_Send16bit(RegDef_SPI_t *pSPIx, uint16_t pTxBuffer)
{

        /* enable module spi */
        SPI_PeripheralEnable(pSPIx, Enable);
        /* wait buffer empty*/
        while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_ALREADY_RESET)
            ;
        /* Check Data sending in 1 time is 8 or 16 bit */

            /* if buffer send = 16bit */
            pSPIx->DR = ((pTxBuffer)); /* pointer cash in 16bit it will tranfer 16bit = 2 lement 8 bit in 1 time*/
        SPI_PeripheralEnable(pSPIx, Disable);

}
