#include "..\STM32F407VG_Library\stm32f407vg.h"
#include "..\STM32F407VG_Library\stm32f407vg_gpio_api.h"
#include "..\STM32F407VG_Library\stm32f407vg_Can_driver.h"

/* CAN1 - PB8 - RX */
/* CAN1 - PB9 - TX */

void CAN_ConfigInitData(void);
void CAN_ConfigTxData(void);
void GPIO_Config(void);
CAN_Handle_t hcan1;
int main(void)
{
    GPIO_Config();
    CAN_ConfigInitData();
    CAN_StartCommunicate(&hcan1);
    CAN_ConfigTxData();
    while (1)
    {


    }
    return 0;
}

void CAN_ConfigInitData(void)
{   
    hcan1.pCANx = CAN1;
    hcan1.CAN_ConfigInitData.CAN_Mode = CAN_Mode_LoopBack;
    hcan1.CAN_ConfigInitData.CAN_BaudratePrescaler = 1;
    hcan1.CAN_ConfigInitData.CAN_SJW = CAN_SJW_1TQ;
    hcan1.CAN_ConfigInitData.CAN_TimeSeg1 = CAN_TimeSeg1_13TQ;
    hcan1.CAN_ConfigInitData.CAN_TimeSeg2 = CAN_TimeSeg2_2TQ;
    hcan1.CAN_ConfigInitData.CAN_TimeTriggerMode = Disable;
    hcan1.CAN_ConfigInitData.CAN_AutomaticBusOff = Disable;
    hcan1.CAN_ConfigInitData.CAN_AutomaticWakeup = Disable;
    hcan1.CAN_ConfigInitData.CAN_NoRetransmission = Enable;
    hcan1.CAN_ConfigInitData.CAN_TransmitFIFOPriority = Disable;
    hcan1.CAN_ConfigInitData.CAN_ReceiveFIFOLocked = Disable;

    CAN_Initialize(&hcan1);
}

void CAN_ConfigTxData(void)
{
    uint32_t Txmailbox;
    uint8_t message[5] = {'H','E','L','L','O'};
    hcan1.CAN_ConfigTxHeader.CAN_IDE = CAN_IDE_StdID;
    hcan1.CAN_ConfigTxHeader.CAN_RTR = CAN_RTR_DataFrame;
    hcan1.CAN_ConfigTxHeader.CAN_StandardID = 0x65D;
    hcan1.CAN_ConfigTxHeader.CAN_DLC = CAN_DLC_5Byte;

    CAN_AddTxMessage(&hcan1,message,&Txmailbox);
    
}

void GPIO_Config(void)
{
    GPIO_Handle_t GpioSPI2;
    GpioSPI2.pGPIOx = GPIOB;
    GpioSPI2.GPIO_Pin_Config.Pin_Moder = Alt_Func_Mode;
    GpioSPI2.GPIO_Pin_Config.Pin_Alternate_Func = AF9;
    GpioSPI2.GPIO_Pin_Config.Pin_Speed = High_speed;
    // GpioSPI2.GPIO_Pin_Config.Pin_OutputType = Output_PushPull;
    GpioSPI2.GPIO_Pin_Config.Pin_PullUp_PullDown = None;
/* CAN1 - PB8 - RX */
/* CAN1 - PB9 - TX */
    /* loop back mode -just config Tx pin CAN*/
    // GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No8;
    // GPIO_Initialize(&GpioSPI2);
    GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No9;
    GPIO_Initialize(&GpioSPI2);

}

