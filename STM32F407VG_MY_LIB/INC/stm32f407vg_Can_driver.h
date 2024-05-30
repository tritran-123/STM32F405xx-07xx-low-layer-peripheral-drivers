#ifndef STM32F407VG_CAN_H
#define STM32F407VG_CAN_H

#include "stm32f407vg.h"
/* Function enable + disable clock module CAN for each Peripherals */
/* bit in register (RCC_APB1ENR) */
#define CAN1EN 25
#define CAN2EN 26
/* Enable clock func */
#define CAN1_Enable_Clock() ((RCC->APB1ENR) |= (Enable << CAN1EN))
#define CAN2_Enable_Clock() ((RCC->APB1ENR) |= (Enable << CAN2EN))
/* Disable clock func */
#define CAN1_Disable_Clock() ((RCC->APB1ENR) &= ~(Enable << CAN1EN))
#define CAN2_Disable_Clock() ((RCC->APB1ENR) &= ~(Enable << CAN2EN))

/* define each location bit field in register */
/*  CAN master control register (CAN_MCR) */
#define CAN_MCR_INRQ    0
#define CAN_MCR_SLEEP   1
#define CAN_MCR_TXFP    2
#define CAN_MCR_RFLM    3
#define CAN_MCR_NART    4
#define CAN_MCR_AWUM    5
#define CAN_MCR_ABOM    6
#define CAN_MCR_TTCM    7
#define CAN_MCR_RESET   15
#define CAN_MCR_DBF     16
/* CAN master status register (CAN_MSR) */
#define CAN_MSR_INAK    0
#define CAN_MSR_SLAK    1
#define CAN_MSR_ERRI    2
#define CAN_MSR_WKUI    3
#define CAN_MSR_SLAKI   4
#define CAN_MSR_TXM     8
#define CAN_MSR_RXM     9
#define CAN_MSR_SAMP    10
#define CAN_MSR_RX      11
/*  CAN transmit status register (CAN_TSR) */
#define CAN_TSR_RQCP0    0
#define CAN_TSR_TXOK0    1
#define CAN_TSR_ALST0    2
#define CAN_TSR_TERR0    3
#define CAN_TSR_ABRQ0    7
#define CAN_TSR_RQCP1    8
#define CAN_TSR_TXOK1    9
#define CAN_TSR_ALST1    10
#define CAN_TSR_TERR1    11
#define CAN_TSR_ABRQ1    15
#define CAN_TSR_RQCP2    16
#define CAN_TSR_TXOK2    17
#define CAN_TSR_ALST2    18
#define CAN_TSR_TERR2    19
#define CAN_TSR_ABRQ2    23
#define CAN_TSR_CODE     24
#define CAN_TSR_TME0     26
#define CAN_TSR_TME1     27
#define CAN_TSR_TME2     28
#define CAN_TSR_LOW0     29
#define CAN_TSR_LOW1     30
#define CAN_TSR_LOW2     31
/* CAN receive FIFO 0 register (CAN_RF0R) */
#define CAN_RF0R_FMP0     0
#define CAN_RF0R_FULL0    3
#define CAN_RF0R_FOVR0    4
#define CAN_RF0R_RFOM0    5
/* CAN receive FIFO 1 register (CAN_RF1R) */
#define CAN_RF1R_FMP1     0
#define CAN_RF1R_FULL1    3
#define CAN_RF1R_FOVR1    4
#define CAN_RF1R_RFOM1    5
/*  CAN bit timing register (CAN_BTR) */
#define CAN_BTR_BRP     0
#define CAN_BTR_TS1     16
#define CAN_BTR_TS2     20
#define CAN_BTR_SJW     24
#define CAN_BTR_LBKM    30
#define CAN_BTR_SILM    31
/*  CAN TX mailbox identifier register (CAN_TIxR) (x=0..2) */
#define CAN_TIxR_TXRQ                   0
#define CAN_TIxR_RTR                    1
#define CAN_TIxR_IDE                    2
#define CAN_TIxR_EXID0_12               3
#define CAN_TIxR_EXID17_13              16
#define CAN_TIxR_STID10_0_EXID28_18     21
/*  CAN mailbox data length control and time stamp register (CAN_TDTxR) (x=0..2) */
#define CAN_TDTxR_DLC   0
/*  CAN mailbox data low register (CAN_TDLxR) (x=0..2) */
#define CAN_TDLxR_DATA0    0
#define CAN_TDLxR_DATA1    8
#define CAN_TDLxR_DATA2    16
#define CAN_TDLxR_DATA3    24
/* CAN mailbox data high register (CAN_TDHxR) (x=0..2) */
#define CAN_TDHxR_DATA4    0
#define CAN_TDHxR_DATA5    8
#define CAN_TDHxR_DATA6    16
#define CAN_TDHxR_DATA7    24
/* CAN receive FIFO mailbox identifier register (CAN_RIxR) */
#define CAN_RIxR_TXRQ                   0
#define CAN_RIxR_RTR                    1
#define CAN_RIxR_IDE                    2
#define CAN_RIxR_EXID0_12               3
#define CAN_RIxR_EXID17_13              16
#define CAN_RIxR_STID10_0_EXID28_18     21
/* CAN receive FIFO mailbox data length control and time stamp register (CAN_RDTxR)*/
#define CAN_RDTxR_DLC   0
#define CAN_RDTxR_FMI   8
/*   CAN receive FIFO mailbox data low register (CAN_RDLxR)) */
#define CAN_RDLxR_DATA0    0
#define CAN_RDLxR_DATA1    8
#define CAN_RDLxR_DATA2    16
#define CAN_RDLxR_DATA3    24
/* CAN receive FIFO mailbox data high register (CAN_RDHxR) */
#define CAN_RDHxR_DATA4    0
#define CAN_RDHxR_DATA5    8
#define CAN_RDHxR_DATA6    16
#define CAN_RDHxR_DATA7    24
/*  CAN filter master register (CAN_FMR) */
#define CAN_FMR_FINIT   0
#define CAN_FMR_CAN2SB  8




/* define information about status each bit field */
    /* Init DATA */
        /* CAN Mode*/
#define CAN_Mode_LoopBack 0
#define CAN_Mode_Silent   1  
#define CAN_Mode_Normal   2  
        /* CAN SJW */
#define CAN_SJW_1TQ         0     /*!< 1 time quantum */
#define CAN_SJW_2TQ         1     /*!< 2 time quantum */
#define CAN_SJW_3TQ         2     /*!< 3 time quantum */
        /* CAN TIME SEGMENT 1*/
#define CAN_TimeSeg1_1TQ    0
#define CAN_TimeSeg1_2TQ    1
#define CAN_TimeSeg1_3TQ    2
#define CAN_TimeSeg1_4TQ    3
#define CAN_TimeSeg1_5TQ    4
#define CAN_TimeSeg1_6TQ    5
#define CAN_TimeSeg1_7TQ    6
#define CAN_TimeSeg1_8TQ    7
#define CAN_TimeSeg1_9TQ    8
#define CAN_TimeSeg1_10TQ   9
#define CAN_TimeSeg1_11TQ   10
#define CAN_TimeSeg1_12TQ   11
#define CAN_TimeSeg1_13TQ   12
#define CAN_TimeSeg1_14TQ   13
#define CAN_TimeSeg1_15TQ   14
#define CAN_TimeSeg1_16TQ   15
        /* CAN TIME SEGMENT 2*/
#define CAN_TimeSeg2_1TQ    0
#define CAN_TimeSeg2_2TQ    1
#define CAN_TimeSeg2_3TQ    2
#define CAN_TimeSeg2_4TQ    3
#define CAN_TimeSeg2_5TQ    4
#define CAN_TimeSeg2_6TQ    5
#define CAN_TimeSeg2_7TQ    6
#define CAN_TimeSeg2_8TQ    7
        /* CAN Baudrateprescaler */
        /* number user set - 1 assign into bit field BRP in BTR reg */
    /* Init TxHeader */
        /*  CAN IDE */
#define CAN_IDE_StdID 0
#define CAN_IDE_ExdID 1
        /* CAN RTR */
#define CAN_RTR_DataFrame   0
#define CAN_RTR_RemoteFrame 1
        /* CAN DLC */
#define CAN_DLC_1Byte 1
#define CAN_DLC_2Byte 2
#define CAN_DLC_3Byte 3
#define CAN_DLC_4Byte 4
#define CAN_DLC_5Byte 5
#define CAN_DLC_6Byte 6
#define CAN_DLC_7Byte 7
#define CAN_DLC_8Byte 8

    /* Init RxHeader */
#define CAN_RX_FIFO0  0
#define CAN_RX_FIFO1  1                                                                  

    /* Config Filter */
        /* FM1R */
#define CAN_FilterMode_IDMaskMode  0
#define CAN_FilterMode_IDListMode  1
        /* FS1R */
#define CAN_FilterScale_Dual16bit     0
#define CAN_FilterScale_Single32bit   1
        /* FFA1R */
#define CAN_FilterAssignFiFO_FIFO0     0
#define CAN_FilterAssignFiFO_FIFO1     1
        /* FA1R */
#define CAN_FilterActive_NotActive  0
#define CAN_FilterActive_Active     1



typedef struct 
{
    uint16_t CAN_BaudratePrescaler;
    uint8_t CAN_Mode;
    uint8_t CAN_SJW;
    uint8_t CAN_TimeSeg1;
    uint8_t CAN_TimeSeg2;
    uint8_t CAN_TimeTriggerMode;
    uint8_t CAN_AutomaticBusOff;
    uint8_t CAN_AutomaticWakeup;
    uint8_t CAN_NoRetransmission;
    uint8_t CAN_ReceiveFIFOLocked;
    uint8_t CAN_TransmitFIFOPriority;

} CAN_Config_Init_Data_t;

typedef struct 
{
    uint16_t CAN_StandardID;
    uint32_t CAN_ExtendedID;
    uint8_t CAN_IDE;
    uint8_t CAN_RTR;
    uint32_t CAN_DLC;

} CAN_Config_Header_Tx_t;

typedef struct 
{
   /* similar Txheader , take data from fifo receive reg store into struct */
    uint16_t CAN_StandardID;
    uint32_t CAN_ExtendedID;
    uint8_t CAN_IDE;
    uint8_t CAN_RTR;
    uint32_t CAN_DLC;
    uint32_t FilterMatchIndex;
} CAN_Config_Header_Rx_t;

typedef struct 
{
    /* CAN_FilterBank */
    uint16_t CAN_FilterIDHigh;
    uint16_t CAN_FilterIDLow;
    uint16_t CAN_FilterMaskIDHigh;
    uint16_t CAN_FilterMaskIDLow;
    uint32_t CAN_FilterBankNumber;
    /* CAN_FMR*/
    uint8_t CAN_FilterStartNumberCAN2;
    /* CAN_FMR1 */
    uint32_t CAN_FilterMode;
    /* CAN_FS1R */
    uint32_t CAN_FilterScale;
    /* CAN_FFA1R */
    uint32_t CAN_FilterAssignFIFO;
    /* CAN_FA1R */
    uint32_t CAN_FilterActive;

} CAN_Config_Filter_t;

typedef struct 
{
    RegDef_CANx_t *pCANx;
    CAN_Config_Init_Data_t CAN_ConfigInitData;
    CAN_Config_Header_Tx_t CAN_ConfigTxHeader;
    CAN_Config_Header_Rx_t CAN_ConfigRxHeader;
    CAN_Config_Filter_t    CAN_ConfigFilter;
} CAN_Handle_t;

/* prototype func */
void CAN_PeripheralClockControl(RegDef_CANx_t *pCANx, uint8_t EnOrDis);
void CAN_Initialize(CAN_Handle_t *pCANxHandle);
void CAN_AddTxMessage(CAN_Handle_t *pCANxHandle,uint8_t TxData[],uint32_t *pTxMailBox);
void CAN_StartCommunicate(CAN_Handle_t *pCANxHandle);
void CAN_GetRxMessage(CAN_Handle_t *pCANxHandle,uint8_t RxData[],uint32_t RxFIFO);
uint32_t CAN_CheckRxFIFOMessagePending(CAN_Handle_t *pCANxHandle,uint32_t RxFIFO); /* return value to determine whether fifo user check already had data */
void CAN_ConfigFilter(CAN_Handle_t *pCANxHandle);

#endif