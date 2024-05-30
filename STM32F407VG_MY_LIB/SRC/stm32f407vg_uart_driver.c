#include "stm32f407vg_uart_driver.h"

void USART_PeripheralClockControl(RegDef_USARTx_t *pUSARTx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        if (pUSARTx == USART1)
        {
            USART1_Enable_Clock();
        }
        else if (pUSARTx == USART2)
        {
            USART2_Enable_Clock();
        }
        else if (pUSARTx == USART3)
        {
            USART3_Enable_Clock();
        }
        else if (pUSARTx == UART4)
        {
            UART4_Enable_Clock();
        }
        else if (pUSARTx == UART5)
        {
            UART5_Enable_Clock();
        }
        else if (pUSARTx == USART6)
        {
            USART6_Enable_Clock();
        }
    }
    else if (EnOrDis == Disable)
    {
        if (pUSARTx == USART1)
        {
            USART1_Disable_Clock();
        }
        else if (pUSARTx == USART2)
        {
            USART2_Disable_Clock();
        }
        else if (pUSARTx == USART3)
        {
            USART3_Disable_Clock();
        }
        else if (pUSARTx == UART4)
        {
            UART4_Disable_Clock();
        }
        else if (pUSARTx == UART5)
        {
            UART5_Disable_Clock();
        }
        else if (pUSARTx == USART6)
        {
            USART6_Disable_Clock();
        }
    }
}

void USART_Initialize(USART_Handle_t *pUSARTxHandle)
{
    /* enable clock in module usart chanel x */
    USART_PeripheralClockControl(pUSARTxHandle->pUSARTx, Enable);
    /* Declare variable temp to assign each bit field's value (total 15/32bit) in register Control register 1 (USART_CR1) */
    uint32_t tempreg = 0;
    /* assign each bit field's value (total 15/32bit) in register  Control register 1 (USART_CR1) */
    /* Set Mode */
    if (pUSARTxHandle->USART_Config.USART_Mode == USART_Mode_Only_RX)
    {
        tempreg |= (Enable << USART_CR1_RE);
    }
    else if (pUSARTxHandle->USART_Config.USART_Mode == USART_Mode_Only_TX)
    {
        tempreg |= (Enable << USART_CR1_TE);
    }
    else if (pUSARTxHandle->USART_Config.USART_Mode == USART_Mode_TXRX)
    {
        tempreg |= ((Enable << USART_CR1_RE) | (Enable << USART_CR1_TE));
    }
    /* Set WordLenght */
    tempreg |= (pUSARTxHandle->USART_Config.USART_WordLenght << USART_CR1_M);
    /* Set Parity */
    if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_EN_EVEN)
    {
        // tempreg &= ~(Set << USART_CR1_PS);
        tempreg |= (Enable << USART_CR1_PCE);
    }
    else if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_EN_ODD)
    {
        tempreg |= (Set << USART_CR1_PS);
        tempreg |= (Enable << USART_CR1_PCE);
    }
    else if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
    {
        tempreg &= ~(Enable << USART_CR1_PCE);
    }
    /* assign all value in CR1 */
    pUSARTxHandle->pUSARTx->CR1 = tempreg;
    /* End CR1 */
    tempreg = 0;
    /* assign each bit field's value (total 15/32bit) in register  Control register 2 (USART_CR2) */
    /* Set Stop bit*/
    tempreg |= (pUSARTxHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
    /* assign all value in CR2 */
    pUSARTxHandle->pUSARTx->CR2 = tempreg;
    /* End CR2 */
    tempreg = 0;
    /* assign each bit field's value (total 15/32bit) in register  Control register 3 (USART_CR3) */
    /* Set HWFlow */
    if (pUSARTxHandle->USART_Config.USART_HWFlowControl == USART_HWFlowControl_CTS)
    {
        tempreg |= (Enable << USART_CR3_CTSE);
    }
    else if (pUSARTxHandle->USART_Config.USART_HWFlowControl == USART_HWFlowControl_RTS)
    {
        tempreg |= (Enable << USART_CR3_RTSE);
    }
    else if (pUSARTxHandle->USART_Config.USART_HWFlowControl == USART_HWFlowControl_CTS_RTS)
    {
        tempreg |= (Enable << USART_CR3_CTSE);
        tempreg |= (Enable << USART_CR3_RTSE);
    }
    else if (pUSARTxHandle->USART_Config.USART_HWFlowControl == USART_HWFlowControl_NONE)
    {
        tempreg &= ~(Enable << USART_CR3_CTSE);
        tempreg &= ~(Enable << USART_CR3_RTSE);
    }
    /* assign all value in CR3 */
    pUSARTxHandle->pUSARTx->CR3 = tempreg;
    /* Call function to set Baudrate */
    USART_SetBaudRate(pUSARTxHandle->pUSARTx, pUSARTxHandle->USART_Config.USART_BaudRate);
}

void USART_SetBaudRate(RegDef_USARTx_t *pUSARTx, uint32_t BaudRate)
{
    /* In default MCU used main clock is HSI = 16MHz in 2 Bus APB1 APB2 = 16MHz*/
    /* if use funtion SYS_Config_Clock_Tree() to set max clock in each Bus (APB1 = 42MHz) (APB2 = 84MHz)*/
    uint32_t fclk;
    uint32_t DetermineSWS = RCC->CFGR; /*Bits 3:2 SWS: System clock switch status determine main system clock is HSI or PLL */
    if ((DetermineSWS >> 2) & 0x3)
    {
        /* SYS clock is PLL 10: PLL used as the system clock */
        if (pUSARTx == USART1 || pUSARTx == USART6)
        {
            fclk = 84000000; /* USART1 USART6 in Bus APB2 (APB2 = 84MHz) */
        }
        else
        {
            fclk = 42000000; /*another chanel USART2345 in APB1 (APB1 = 42MHz) */
        }
    }
    else
    {
        /* 00: HSI oscillator used as the system clock */
        fclk = 16000000; /* 16MHz in 2 bus APB1 APB2 */
    }
    /* Determine OVER8 */
    /* Read bit OVER8 in reg USART_CR1
    Bit 15 OVER8: Oversampling mode
    0: oversampling by 16
    1: oversampling by 8*/
    uint32_t DivMantissa, Over8, Over8_Sampling;
    double UsartDiv, DivFraction_f;                  /* perform value fraction in type double */
    Over8 = (pUSARTx->CR1 >> USART_CR1_OVER8) & 0x1; /* read bit 15 in reg */
    Over8_Sampling = 8 * (2 - Over8);
    UsartDiv = ((double)fclk) / (Over8_Sampling * BaudRate);

    /* determine Mantissa and Fraction (RM page 979) */
    DivFraction_f = (UsartDiv - (uint32_t)UsartDiv) * Over8_Sampling; /* take decimal part and *over8_sampling (8 or 16) */
    uint32_t DivFraction_round = (uint32_t)round(DivFraction_f);      /* round this value to nearest int number */
    uint8_t carry_flag = (DivFraction_round == Over8_Sampling);       /* if DivFraction_round perform overflow value :  overflow of DIV_frac[3:0](Over8 = 0) :  overflow of the DIV_frac[2:0] (Over8 = 1)*/

    uint32_t DivFraction = DivFraction_round % Over8_Sampling; /* if value div fration < over8_sampling corrossponding -> keep it value
                                                                else(overflow) reset div fraction to 0 and increase mantissa to (carry_flag ) unit*/
    DivMantissa = (uint32_t)UsartDiv + carry_flag;
    /* Set 12bit perform Value Mantissa and 4 bit Fraction into USART_BRR */
    uint32_t tempreg = 0;
    tempreg |= (DivFraction);
    tempreg |= (DivMantissa << USART_BRR_DIV_Mantissa);
    /*  assign all value in USART_BRR  */
    pUSARTx->BRR = tempreg;
}

void USART_PeripheralControl(RegDef_USARTx_t *pUSARTx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        pUSARTx->CR1 |= (Enable << USART_CR1_UE);
    }
    else
    {
        pUSARTx->CR1 &= ~(Enable << USART_CR1_UE);
    }
}

uint8_t USART_GetFlagStatus(RegDef_USARTx_t *pUSARTx, uint32_t FlagName)
{
    if (pUSARTx->SR & (FlagName))
    {
        return FLAG_ALREADY_SET;
    }
    return FLAG_ALREADY_RESET;
}

void USART_SendData(USART_Handle_t *pUSARTxHandle, uint8_t *TxBuffer, uint32_t Len)
{
    /* full function for cases check parity and wordlenght */

    for (uint32_t i = 0; i < Len; i++)
    {

        /* Wait TXE: Transmit data register empty */
        while (!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_TXE))
            ;
        /* check wordlenght to determine total data bit transmit */
        if (pUSARTxHandle->USART_Config.USART_WordLenght == USART_WordLenght_9DataBits)
        {
            /* 9 bit data */
            /* Check parity bit */
            if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
            {
                /* don't use paritybit => 9bit including 9bit data */
                /* take 9bit data in DR ( &0x01FF) reg into buffer transmit (extract datatype uint8 to uint16 to store 1 frame data ) */
                pUSARTxHandle->pUSARTx->DR = (*((uint16_t *)TxBuffer) & (uint16_t)0x01FF);
                /* need 16bit to store 1 frame data so point to next 2 element 8bit to store next data */
                TxBuffer += 2;
            }
            else
            {
                /*  use paritybit => 9bit including 8bit data and 1bit parity */
                /* take 8bit data in DR ( &0xFF) into buffer */
                pUSARTxHandle->pUSARTx->DR = (*(TxBuffer) & (uint8_t)0xFF);
                TxBuffer++;
            }
        }
        else
        {
            /* 8 bit data */
            /* check parity bit */
            if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
            {
                /* don't use paritybit => 8bit including 8bit data */
                pUSARTxHandle->pUSARTx->DR = (*(TxBuffer) & (uint8_t)0xFF);
            }
            else
            {
                /* 8bit frame = 7bitdata + 1bit parity */
                pUSARTxHandle->pUSARTx->DR = (*(TxBuffer) & (uint8_t)0x7F);
            }
            TxBuffer++;
        }
    }
    /* Wait TC: Transmission complete */
    while (!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_TC))
        ;
}

void USART_SendChar(RegDef_USARTx_t *pUSARTx, unsigned char TxBuffer)
{
    /* Simple test for data 8 bit - No parity */

    /* Wait TXE: Transmit data register empty */
    while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE))
        ;

    /* Send data 8bit in DR reg */
    pUSARTx->DR = (unsigned char)TxBuffer;
    // TxBuffer++;

    /* Wait TC: Transmission complete */
    // while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTxHandle, uint8_t *RxBuffer, uint32_t Len)
{
    /* full function for cases check parity and wordlenght */

    for (uint32_t i = 0; i < Len; i++)
    {
        /* Wait flag RXNE: Read data register not empty */
        while (!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_RXNE))
            ;
        /* check wordlenght to determine total data bit receive */
        if (pUSARTxHandle->USART_Config.USART_WordLenght == USART_WordLenght_9DataBits)
        {
            /* 9 bit data */
            /* Check parity bit */
            if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
            {
                /* don't use paritybit => 9bit including 9bit data */
                /* take 9bit data in DR ( &0x01FF) reg into buffer receive (extract datatype uint8 to uint16 to store 1 frame data ) */
                *((uint16_t *)RxBuffer) = ((pUSARTxHandle->pUSARTx->DR) & (uint16_t)0x01FF);
                /* need 16bit to store 1 frame data so point to next 2 element 8bit to store next data */
                RxBuffer += 2;
            }
            else
            {
                /*  use paritybit => 9bit including 8bit data and 1bit parity */
                /* take 8bit data in DR ( &0xFF) into buffer */
                *(RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0xFF);
                RxBuffer++;
            }
        }
        else
        {
            /* 8 bit data */
            /* check parity bit */
            if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
            {
                /* don't use paritybit => 8bit including 8bit data */
                *(RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0xFF);
            }
            else
            {
                /* 8bit frame = 7bitdata + 1bit parity */
                *(RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0x7F);
            }
            RxBuffer++;
        }
    }
}

/* INTERRUPT FUNCT */
uint8_t USART_SendDataInterruptInit(USART_Handle_t *pUSARTxHandle, uint8_t *TxBuffer, uint32_t Len)
{
    uint8_t state = pUSARTxHandle->TxState;
    if (state != USART_BUSY_IN_TX)
    {
        /* Similar init assign address data like SPI module interrupt */
        pUSARTxHandle->TxBuffer = TxBuffer;
        pUSARTxHandle->TxLen = Len;
        pUSARTxHandle->TxState = USART_BUSY_IN_TX;
        /* Enable  Bit 7 TXEIE: TXE interrupt enable */
        pUSARTxHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
        /* Enable  Bit 6 TCIE: Transmission complete interrupt enable */
        pUSARTxHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }
    return state;
}

uint8_t USART_ReceiveDataInterruptInit(USART_Handle_t *pUSARTxHandle, uint8_t *RxBuffer, uint32_t Len)
{
    uint8_t state = pUSARTxHandle->RxState;
    if (state != USART_BUSY_IN_RX)
    {
        /* Similar init assign address data like SPI module interrupt */
        pUSARTxHandle->RxBuffer = RxBuffer;
        pUSARTxHandle->RxLen = Len;
        pUSARTxHandle->RxState = USART_BUSY_IN_RX;

        /* Read to clean DR reg */
        (void)pUSARTxHandle->pUSARTx->DR;

        /* Enable  Bit 5 RXNEIE: RXNE interrupt enable */
        pUSARTxHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }
    return state;
}

void USART_IRQHandling(USART_Handle_t *pUSARTxHandle)
{
    /* Declare 2 temp variable to read bits, determine source interrupt */
    uint32_t temp1 = 0, temp2 = 0;
    /* Tc flag */
    /* Read status bit TC in reg SR and bit TCIE in cr1 */
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_TC));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR1_TCIE));

    if (temp1 && temp2)
    {
        USART_TC_Interrupt_Handle(pUSARTxHandle);
    }

    /* TXE flag */
    /* Read status bit TXE in reg SR and bit TXEIE in cr1 */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_TXE));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR1_TXEIE));

    if (temp1 && temp2)
    {
        USART_TXE_Interrupt_Handle(pUSARTxHandle);
    }

    /* RXNE flag */
    /* Read status bit RXNE in reg SR and bit RXNEIE in cr1 */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_RXNE));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR1_RXNEIE));

    if (temp1 && temp2)
    {
        USART_RXNE_Interrupt_Handle(pUSARTxHandle);
    }

    /* SOURCE interrupt by error frame and detect idle line*/
    /* CTS flag */
    /* Read status bit CTS in reg SR and bit RXNEIE in cr1 */
    /*  Note: This bit is not available for UART4 & UART5. */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_CTS));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR3_CTSE));
    if (temp1 && temp2)
    {
        /* Clear flag */
        pUSARTxHandle->pUSARTx->SR &= ~(1U << USART_SR_CTS);
        /* Call Applicationcallback to notify and implement code */
        USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_CTS);
    }

    /* IDLE flag */
    /* Read status bit IDLE in reg SR and bit IDELEIE in cr1 */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_IDLE));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR1_IDLEIE));
    if (temp1 && temp2)
    {
        /* Clear flag by (an read to the USART_SR register followed by a read to the USART_DR register).*/
        temp1 = pUSARTxHandle->pUSARTx->SR;
        /* Call Applicationcallback to notify and implement code */
        USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_IDLE);
    }

    /* Overrun Error Detected flag */
    /* Read status bit ORE in reg SR and bit RXNEIE in cr1 */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->SR & (Set << USART_SR_ORE));
    temp2 = (pUSARTxHandle->pUSARTx->CR1 & (Set << USART_CR1_RXNEIE));
    if (temp1 && temp2)
    {
        /* Call Applicationcallback to notify and implement code */
        USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_ORE);
    }

    /* Noise Flag, Overrun error and Framing Error in multibuffer communication flag */
    /* Read status bit NF or ORE or FE in reg SR and bit EIE in cr3 */
    temp1 = 0;
    temp2 = 0;
    temp1 = (pUSARTxHandle->pUSARTx->CR3 & (Set << USART_CR3_EIE));
    if (temp1)
    {
        /* Read reg SR and check 3 bit FE NF ORE*/
        temp2 = pUSARTxHandle->pUSARTx->SR;
        if (temp2 & (Set << USART_SR_ORE))
        {
            /* Call Applicationcallback to notify and implement code */
            USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_ORE);
        }
        if (temp2 & (Set << USART_SR_NF))
        {
            /* Call Applicationcallback to notify and implement code */
            USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_NF);
        }
        if (temp2 & (Set << USART_SR_FE))
        {
            /* Call Applicationcallback to notify and implement code */
            USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_FE);
        }
    }
}

void USART_TC_Interrupt_Handle(USART_Handle_t *pUSARTxHandle)
{
    /* check status module usart */
    if (pUSARTxHandle->TxState == USART_BUSY_IN_TX)
    {
        /* if already transmit all data, len = 0*/
        if (!pUSARTxHandle->TxLen)
        {
            /* clear status bit TC in SR */
            pUSARTxHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
            /* clear bit */
            pUSARTxHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
            /* already transmit data -> Returns variables to default values */
            pUSARTxHandle->TxState = USART_READY;
            pUSARTxHandle->TxBuffer = NULL;
            pUSARTxHandle->TxLen = 0;
        }
        /* Call function ApplicationEventCallBack()*/
        USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_TX_CMPLT);

    }
}

void USART_TXE_Interrupt_Handle(USART_Handle_t *pUSARTxHandle)
{
    /* check status module usart */
    if (pUSARTxHandle->TxState == USART_BUSY_IN_TX)
    {
        /* check len data and trasmit similar func USART_SendData() */
        if (pUSARTxHandle->TxLen > 0)
        {
            /* check wordlenght to determine total data bit transmit */
            if (pUSARTxHandle->USART_Config.USART_WordLenght == USART_WordLenght_9DataBits)
            {
                /* 9 bit data */
                /* Check parity bit */
                if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
                {
                    /* don't use paritybit => 9bit including 9bit data */
                    /* take 9bit data in DR ( &0x01FF) reg into buffer transmit (extract datatype uint8 to uint16 to store 1 frame data ) */
                    pUSARTxHandle->pUSARTx->DR = (*((uint16_t *)pUSARTxHandle->TxBuffer) & (uint16_t)0x01FF);
                    /* need 16bit to store 1 frame data so point to next 2 element 8bit to store next data */
                    pUSARTxHandle->TxBuffer += 2;
                    pUSARTxHandle->TxLen -= 2;
                }
                else
                {
                    /*  use paritybit => 9bit including 8bit data and 1bit parity */
                    /* take 8bit data in DR ( &0xFF) into buffer */
                    pUSARTxHandle->pUSARTx->DR = (*(pUSARTxHandle->TxBuffer) & (uint8_t)0xFF);
                    pUSARTxHandle->TxBuffer++;
                    pUSARTxHandle->TxLen--;
                }
            }
            else
            {
                /* 8 bit data */
                /* check parity bit */
                if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
                {
                    /* don't use paritybit => 8bit including 8bit data */
                    pUSARTxHandle->pUSARTx->DR = (*(pUSARTxHandle->TxBuffer) & (uint8_t)0xFF);
                }
                else
                {
                    /* 8bit frame = 7bitdata + 1bit parity */
                    pUSARTxHandle->pUSARTx->DR = (*(pUSARTxHandle->TxBuffer) & (uint8_t)0x7F);
                }
                pUSARTxHandle->TxBuffer++;
                pUSARTxHandle->TxLen--;
            }
        }
        if (pUSARTxHandle->TxLen == 0)
        {
            pUSARTxHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
        }
    }
}

void USART_RXNE_Interrupt_Handle(USART_Handle_t *pUSARTxHandle)
{
    /* check status module usart */
    if (pUSARTxHandle->RxState == USART_BUSY_IN_TX)
    {
        /* check len data and trasmit similar func USART_SendData() */
        if (pUSARTxHandle->RxLen > 0)
        {
            /* check wordlenght to determine total data bit receive */
            if (pUSARTxHandle->USART_Config.USART_WordLenght == USART_WordLenght_9DataBits)
            {
                /* 9 bit data */
                /* Check parity bit */
                if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
                {
                    /* don't use paritybit => 9bit including 9bit data */
                    /* take 9bit data in DR ( &0x01FF) reg into buffer receive (extract datatype uint8 to uint16 to store 1 frame data ) */
                    *((uint16_t *)(pUSARTxHandle->RxBuffer)) = ((pUSARTxHandle->pUSARTx->DR) & (uint16_t)0x01FF);
                    /* need 16bit to store 1 frame data so point to next 2 element 8bit to store next data */
                    pUSARTxHandle->RxBuffer += 2;
                    pUSARTxHandle->RxLen -= 2;
                }
                else
                {
                    /*  use paritybit => 9bit including 8bit data and 1bit parity */
                    /* take 8bit data in DR ( &0xFF) into buffer */
                    *(pUSARTxHandle->RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0xFF);
                    pUSARTxHandle->RxBuffer++;
                    pUSARTxHandle->RxLen--;
                }
            }
            else
            {
                /* 8 bit data */
                /* check parity bit */
                if (pUSARTxHandle->USART_Config.USART_ParityControl == USART_ParityControl_DIS)
                {
                    /* don't use paritybit => 8bit including 8bit data */
                    *(pUSARTxHandle->RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0xFF);
                }
                else
                {
                    /* 8bit frame = 7bitdata + 1bit parity */
                    *(pUSARTxHandle->RxBuffer) = (pUSARTxHandle->pUSARTx->DR & (uint8_t)0x7F);
                }
                pUSARTxHandle->RxBuffer++;
                pUSARTxHandle->RxLen--;
            }
        }
        /* if already receive all data, len = 0*/
        if (!pUSARTxHandle->RxLen)
        {
            /* clear status bit RXNEIE in cr1 */
            pUSARTxHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
            /* already receive data -> Returns variables to default values */
            pUSARTxHandle->RxState = USART_READY;
            pUSARTxHandle->RxBuffer = NULL;
            pUSARTxHandle->RxLen = 0;
        }
        /* Call function ApplicationEventCallBack()*/
        USART_ApplicationEventCallBack(pUSARTxHandle, USART_EVENT_RX_CMPLT);

    }
}

void USART_ApplicationEventCallBack(USART_Handle_t *pUSARTxHandle, uint8_t ApEvent)
{
    if(ApEvent == USART_EVENT_TX_CMPLT)
    {
        
    }
    else if(ApEvent == USART_EVENT_RX_CMPLT)
    {

    }
}
