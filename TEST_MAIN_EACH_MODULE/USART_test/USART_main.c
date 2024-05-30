#include "..\STM32F407VG_Library\stm32f407vg_gpio_api.h"
#include "..\STM32F407VG_Library\stm32f407vg_uart_driver.h"
#include "..\STM32F407VG_Library\stm32f407vg_ClockConfig.h"
#include "..\STM32F407VG_Library\stm32f407vg_Interrupt.h"
#include <stdio.h>
#include <string.h>

USART_Handle_t USARTHandle;         /* Declare variable type struct to handle data in module usart */
char test[] = "Hello World\n";   /* string need to transmit */

/*prototype function*/
void USART_Config(void);
void GPIO_Config(void);

int main(void)
{

    // SYS_Config_Clock_Tree();
    USART_Config();
    GPIO_Config();
    // NVIC_IRQInterruptConfig(IRQNumber_USART2, Enable);
    USART_PeripheralControl(USARTHandle.pUSARTx, Enable);
    // USART_SendDataInterruptInit(&USARTHandle, (uint8_t *)test, strlen(test));

    while (1)
    {
        // while(USART_ReceiveDataInterruptInit(&USARTHandle,receiverbuffer,20) != USART_READY);
        // USART_SendDataInterruptInit(&USARTHandle,receiverbuffer,20);
        USART_SendData(&USARTHandle, (uint8_t *)test, strlen(test));
        while (1)
            ;
    }

    return 0;
}

void USART_Config(void)
{
    USARTHandle.pUSARTx = USART2;
    USARTHandle.USART_Config.USART_BaudRate = USART_BaudRate_115200;
    USARTHandle.USART_Config.USART_Mode = USART_Mode_Only_TX;
    USARTHandle.USART_Config.USART_NoOfStopBits = USART_NoOfStopBits_1StopBits;
    USARTHandle.USART_Config.USART_ParityControl = USART_ParityControl_DIS;
    USARTHandle.USART_Config.USART_HWFlowControl = USART_HWFlowControl_NONE;
    USARTHandle.USART_Config.USART_WordLenght = USART_WordLenght_8DataBits;

    USART_Initialize(&USARTHandle);
}

void GPIO_Config(void)
{
    GPIO_Handle_t GpioUSART2;
    GpioUSART2.pGPIOx = GPIOA;
    GpioUSART2.GPIO_Pin_Config.Pin_Moder = Alt_Func_Mode;
    GpioUSART2.GPIO_Pin_Config.Pin_Alternate_Func = AF7;
    GpioUSART2.GPIO_Pin_Config.Pin_Speed = High_speed;
    GpioUSART2.GPIO_Pin_Config.Pin_OutputType = Output_PushPull;
    GpioUSART2.GPIO_Pin_Config.Pin_PullUp_PullDown = Pull_up;
    /*  PA2 : USART2_TX */
    GpioUSART2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No2;
    GPIO_Initialize(&GpioUSART2);
    /*  PA3 : USART2_RX */
    GpioUSART2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No3;
    GPIO_Initialize(&GpioUSART2);
}

// void USART2_IRQHandler(void)
// {
//     USART_IRQHandling(&USARTHandle);
// }
