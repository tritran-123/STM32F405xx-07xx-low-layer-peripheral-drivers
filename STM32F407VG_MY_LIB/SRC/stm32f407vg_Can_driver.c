#include "stm32f407vg_Can_driver.h"

void CAN_PeripheralClockControl(RegDef_CANx_t *pCANx, uint8_t EnOrDis)
{
    if (EnOrDis == Enable)
    {
        if (pCANx == CAN1)
        {
            CAN1_Enable_Clock();
        }
        else
        {
            CAN2_Enable_Clock();
        }
    }
    else
    {
        if (pCANx == CAN1)
        {
            CAN1_Disable_Clock();
        }
        else
        {
            CAN2_Disable_Clock();
        }
    }
}

void CAN_Initialize(CAN_Handle_t *pCANxHandle)
{
    /* enable clock in module can chanel x */
    CAN_PeripheralClockControl(pCANxHandle->pCANx, Enable);

    /* Request initialisation */
    pCANxHandle->pCANx->MCR |= (Enable << CAN_MCR_INRQ);
    /* Wait initialisation acknowledge */
    while (!(pCANxHandle->pCANx->MSR & (Set << CAN_MSR_INAK)))
        ;

    /* Exit from sleep mode */
    pCANxHandle->pCANx->MCR &= ~(1U << CAN_MCR_SLEEP);
    /* Check Sleep mode leave acknowledge */
    while (pCANxHandle->pCANx->MSR & (Set << CAN_MSR_SLAK))
        ;

    /* Declare variable temp to assign each bit field's value in register */
    uint32_t tempreg = 0;
    /* Set the time triggered communication mode */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_TimeTriggerMode << CAN_MCR_TTCM);
    /* Set the automatic bus-off management */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_AutomaticBusOff << CAN_MCR_ABOM);
    /* Set the automatic wake-up mode */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_AutomaticWakeup << CAN_MCR_AWUM);
    /* Set the automatic retransmission */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_NoRetransmission << CAN_MCR_NART);
    /* Set the receive FIFO locked mode */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_ReceiveFIFOLocked << CAN_MCR_RFLM);
    /* Set the transmit FIFO priority */
    tempreg |= (pCANxHandle->CAN_ConfigInitData.CAN_TransmitFIFOPriority << CAN_MCR_TXFP);
    /* assign all value in MCR ??????????????? if assign this value bit INRQ will be clear ??????????????? => doesnt use operation '=' ,use |= to retain bit INRQ */
    pCANxHandle->pCANx->MCR |= tempreg;

    /* Set the bit timing register */
    tempreg = 0;
    tempreg |= ((pCANxHandle->CAN_ConfigInitData.CAN_SJW << CAN_BTR_SJW) | (pCANxHandle->CAN_ConfigInitData.CAN_TimeSeg1 << CAN_BTR_TS1) | (pCANxHandle->CAN_ConfigInitData.CAN_TimeSeg2 << CAN_BTR_TS2) | ((pCANxHandle->CAN_ConfigInitData.CAN_BaudratePrescaler - 1) << CAN_BTR_BRP)); /*  tq = (BRP[9:0]+1) x tPCLK */
    /* assign all value in BTR */
    pCANxHandle->pCANx->BTR |= tempreg;
    /* Set mode into BTR reg */
    /* Bit 31 SILM: Silent mode (debug)
    0: Normal operation
    1: Silent Mode
    Bit 30 LBKM: Loop back mode (debug)
    0: Loop Back Mode disabled
    1: Loop Back Mode enabled*/
    if (pCANxHandle->CAN_ConfigInitData.CAN_Mode == CAN_Mode_Normal)
    {
        pCANxHandle->pCANx->BTR &= ~(0x3 << CAN_BTR_LBKM);
    }
    else if (pCANxHandle->CAN_ConfigInitData.CAN_Mode == CAN_Mode_Silent)
    {
        pCANxHandle->pCANx->BTR |= (0x2 << CAN_BTR_LBKM);
    }
    else
    {
        pCANxHandle->pCANx->BTR |= (0x1 << CAN_BTR_LBKM);
    }
}

void CAN_AddTxMessage(CAN_Handle_t *pCANxHandle, uint8_t TxData[], uint32_t *pTxMailBox)
{
    uint32_t transmitmailbox;
    /* Check that all the Tx mailboxes(0-1-2) are not full */
    if (((pCANxHandle->pCANx->TSR >> CAN_TSR_TME0) & 0x7) != 0x0) /* take 3 bit TM0-1-2, need 1 bit set(at least 1 mailbox free to transmit data)*/
    {
        /* Select an empty transmit mailbox */
        /* take 2bit  CODE[1:0]: Mailbox code
In case at least one transmit mailbox is free, the code value is equal to the number of the
next transmit mailbox free.In case all transmit mailboxes are pending, the code value is equal to the number of the
transmit mailbox with the lowest priority.*/
        transmitmailbox = ((pCANxHandle->pCANx->TSR & 0x3) >> CAN_TSR_CODE);
        /* Store the Tx mailbox */
        *pTxMailBox = 1U << transmitmailbox;

        /* Set up the Id , IDE , RTR in TIxR reg*/
        if (pCANxHandle->CAN_ConfigTxHeader.CAN_IDE == CAN_IDE_StdID)
        {
            pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TIxR = ((pCANxHandle->CAN_ConfigTxHeader.CAN_StandardID << CAN_TIxR_STID10_0_EXID28_18) | (pCANxHandle->CAN_ConfigTxHeader.CAN_IDE << CAN_TIxR_IDE) | (pCANxHandle->CAN_ConfigTxHeader.CAN_RTR << CAN_TIxR_RTR));
        }
        else
        {
            pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TIxR = ((pCANxHandle->CAN_ConfigTxHeader.CAN_ExtendedID << CAN_TIxR_EXID0_12) | (pCANxHandle->CAN_ConfigTxHeader.CAN_IDE << CAN_TIxR_IDE) | (pCANxHandle->CAN_ConfigTxHeader.CAN_RTR << CAN_TIxR_RTR));
        }
        /* Set up the DLC */
        pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TDTxR = (pCANxHandle->CAN_ConfigTxHeader.CAN_DLC << CAN_TDTxR_DLC);
        /* Set up the Transmit Global Time mode - temporary ignored */
        /* Set up the data field */
        /* data low register (CAN_TDLxR) */
        pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TDLxR = (((uint32_t)TxData[0] << CAN_TDLxR_DATA0) | ((uint32_t)TxData[1] << CAN_TDLxR_DATA1) | ((uint32_t)TxData[2] << CAN_TDLxR_DATA2) | ((uint32_t)TxData[3] << CAN_TDLxR_DATA3));
        /*  data high register (CAN_TDHxR)  */
        pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TDHxR = (((uint32_t)TxData[4] << CAN_TDHxR_DATA4) | ((uint32_t)TxData[5] << CAN_TDHxR_DATA5) | ((uint32_t)TxData[6] << CAN_TDHxR_DATA6) | ((uint32_t)TxData[7] << CAN_TDHxR_DATA7));
        /* Request transmission */
        pCANxHandle->pCANx->CANx_Mailbox[transmitmailbox].TIxR |= (Enable << CAN_TIxR_TXRQ);
    }
}

void CAN_StartCommunicate(CAN_Handle_t *pCANxHandle)
{
    /* Request leave initialisation */
    pCANxHandle->pCANx->MCR &= ~(1U << CAN_MCR_INRQ);
    /* Wait the acknowledge */
    while (pCANxHandle->pCANx->MSR & (Set << CAN_MSR_INAK))
        ;
}

/* return value to determine whether fifo user check already had data */
uint32_t CAN_CheckRxFIFOMessagePending(CAN_Handle_t *pCANxHandle, uint32_t RxFIFO)
{
    /* check fifo user choose */
    if (RxFIFO == CAN_RX_FIFO0)
    {
        /* get bit FMP0[1:0]: FIFO 0 message pending and return to determine fifo already had data to get ? */
        return (pCANxHandle->pCANx->RF0R & (0x3 << CAN_RF0R_FMP0));
    }
    else
    {
        return (pCANxHandle->pCANx->RF1R & (0x3 << CAN_RF1R_FMP1));
    }
}

void CAN_GetRxMessage(CAN_Handle_t *pCANxHandle, uint8_t RxData[], uint32_t RxFIFO)
{
    /* check fifo already had data ?*/
    if (RxFIFO == CAN_RX_FIFO0)
    {
        /* loop if doesn't have data*/
        while ((pCANxHandle->pCANx->RF0R & (0x3 << CAN_RF0R_FMP0)) == 0U)
            ;
    }
    else
    {
        while ((pCANxHandle->pCANx->RF1R & (0x3 << CAN_RF1R_FMP1)) == 0U)
            ;
    }

    /* Get the header */
    /* IDE */
    pCANxHandle->CAN_ConfigRxHeader.CAN_IDE = ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RIxR >> CAN_RIxR_IDE) & 0x1);
    /* ID */
    if (pCANxHandle->CAN_ConfigRxHeader.CAN_IDE == CAN_IDE_StdID)
    {
        pCANxHandle->CAN_ConfigRxHeader.CAN_StandardID = ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RIxR >> CAN_RIxR_STID10_0_EXID28_18) & 0x7FF); /* take 11bit std ID*/
    }
    else
    {
        pCANxHandle->CAN_ConfigRxHeader.CAN_ExtendedID = ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RIxR >> CAN_RIxR_EXID0_12) & 0x1FFFFFFF); /* take 29bit ext ID*/
    }
    /* RTR */
    pCANxHandle->CAN_ConfigRxHeader.CAN_RTR = ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RIxR >> CAN_RIxR_RTR) & 0x1);
    /* check DLC condition <= 8byte data lenght */
    if ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RDTxR & 0xF) > CAN_DLC_8Byte)
    {
        pCANxHandle->CAN_ConfigRxHeader.CAN_DLC = CAN_DLC_8Byte;
    }
    else
    {
        pCANxHandle->CAN_ConfigRxHeader.CAN_DLC = (pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RDTxR & 0xF); /* take 4bit DLC */
    }
    /*  FMI[7:0]: Filter match index
 This register contains the index of the filter the message stored in the mailbox passed
through. For more details on identifier filtering*/
    pCANxHandle->CAN_ConfigRxHeader.FilterMatchIndex = ((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RDTxR >> CAN_RDTxR_FMI) & 0xFF);

    /* Get Data */
    for (uint8_t index = 0; index < 8; index++)
    {
        if (index <= 3)
        {
            /* shift xbit corresponding for each DATA in reg and assgin value to array */
            RxData[index] = (uint8_t)((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RDLxR >> (CAN_RDLxR_DATA1 * index)) & 0xFF);
        }
        else
        {
            RxData[index] = (uint8_t)((pCANxHandle->pCANx->CANx_ReceiveFIFO[RxFIFO].RDHxR >> (CAN_RDHxR_DATA5 * (index - 4U))) & 0xFF);
        }
    }

    /* Release the FIFO */
    if (RxFIFO == CAN_RX_FIFO0)
    {
        /*  Bit 5RFOM1: Release FIFO 1 output mailbox */
        pCANxHandle->pCANx->RF0R |= (Set << CAN_RF0R_RFOM0);
    }
    else
    {
        pCANxHandle->pCANx->RF1R |= (Set << CAN_RF1R_RFOM1);
    }
}

void CAN_ConfigFilter(CAN_Handle_t *pCANxHandle)
{
    /* Initialisation mode for the filter - default already set */
    pCANxHandle->pCANx->FMR |= (Enable << CAN_FMR_FINIT);
    /* Select the start filter number of CAN2 slave instance */
    pCANxHandle->pCANx->FMR &= ~(0x3F << CAN_FMR_CAN2SB); /* clear 6bit CAN2SB */
    /* check CAN2SB value in range 0d-28d */
    if(pCANxHandle->CAN_ConfigFilter.CAN_FilterStartNumberCAN2 >= 0U && pCANxHandle->CAN_ConfigFilter.CAN_FilterStartNumberCAN2 <= 28U)
    {
        pCANxHandle->pCANx->FMR |= ((pCANxHandle->CAN_ConfigFilter.CAN_FilterStartNumberCAN2 & 0x3F ) << CAN_FMR_CAN2SB);
    }

    /* DEtermine filter number position user choose into bit position */
    uint32_t FilterNumPos = ((pCANxHandle->CAN_ConfigFilter.CAN_FilterBankNumber));

    /* Check Filter Deactivation before init value */
    pCANxHandle->pCANx->FA1R &= ~(1u << FilterNumPos);
    /* Filter Scale */
    if (pCANxHandle->CAN_ConfigFilter.CAN_FilterScale == CAN_FilterScale_Single32bit)
    {
        /* 32-bit scale for the filter */
         pCANxHandle->pCANx->FS1R |= (1 << FilterNumPos);
        /* 32-bit identifier or First 32-bit identifier */
        pCANxHandle->pCANx->CANx_FilterBank[FilterNumPos].FiR1 = (((pCANxHandle->CAN_ConfigFilter.CAN_FilterIDHigh & 0x0000FFFF) << 16U) | ((pCANxHandle->CAN_ConfigFilter.CAN_FilterIDLow & 0x0000FFFF) << 0U));
        /* 32-bit mask or Second 32-bit identifier */
        pCANxHandle->pCANx->CANx_FilterBank[FilterNumPos].FiR2 = (((pCANxHandle->CAN_ConfigFilter.CAN_FilterMaskIDHigh & 0x0000FFFF) << 16U) | ((pCANxHandle->CAN_ConfigFilter.CAN_FilterMaskIDLow & 0x0000FFFF) << 0U));
    }
    
    if (pCANxHandle->CAN_ConfigFilter.CAN_FilterScale == CAN_FilterScale_Dual16bit)
    {
        /* 16dual-bit scale for the filter */
         pCANxHandle->pCANx->FS1R &= ~(1U << FilterNumPos);
        /* First 16-bit identifier and First 16-bit mask */
        pCANxHandle->pCANx->CANx_FilterBank[FilterNumPos].FiR1 = (((pCANxHandle->CAN_ConfigFilter.CAN_FilterMaskIDLow & 0x0000FFFF) << 16U) | ((pCANxHandle->CAN_ConfigFilter.CAN_FilterIDLow & 0x0000FFFF) << 0U));
        /* Second 16-bit identifier and Second 16-bit mask */
        pCANxHandle->pCANx->CANx_FilterBank[FilterNumPos].FiR2 = (((pCANxHandle->CAN_ConfigFilter.CAN_FilterMaskIDHigh & 0x0000FFFF) << 16U) | ((pCANxHandle->CAN_ConfigFilter.CAN_FilterIDHigh & 0x0000FFFF) << 0U));
    }

    /* Filter Mode */
    if(pCANxHandle->CAN_ConfigFilter.CAN_FilterMode == CAN_FilterMode_IDListMode)
    {
        pCANxHandle->pCANx->FM1R |= (1 << FilterNumPos);
    }
    else
    {
        pCANxHandle->pCANx->FM1R &= ~(1U << FilterNumPos);
    }
    /* Filter FIFO assignment */
    if(pCANxHandle->CAN_ConfigFilter.CAN_FilterAssignFIFO == CAN_FilterAssignFiFO_FIFO1)
    {
        pCANxHandle->pCANx->FFA1R |= (1 << FilterNumPos);
    }
    else
    {
        pCANxHandle->pCANx->FFA1R &= ~(1U << FilterNumPos);
    }   
    /* Filter activation */
    if(pCANxHandle->CAN_ConfigFilter.CAN_FilterActive == CAN_FilterActive_Active)
    {
        pCANxHandle->pCANx->FA1R |= (1 << FilterNumPos);
    }

    /* Leave the initialisation mode for the filter */
    pCANxHandle->pCANx->FMR &= ~(1U << CAN_FMR_FINIT);

}   