#include "stm32f407vg_gpio_api.h"
#include "stm32f407vg_Interrupt.h"
#include "stm32f407vg_ClockConfig.h"
#include "stm32f407vg_uart_driver.h"
void EXTI0_IRQHandler(void);

uint8_t flag = 0;
int main()
{   
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No14;
    GpioLed.GPIO_Pin_Config.Pin_Moder = GPOutput_Mode;
    GpioLed.GPIO_Pin_Config.Pin_Speed = Medium_speed;
    GpioLed.GPIO_Pin_Config.Pin_OutputType = Output_PushPull;
    GpioLed.GPIO_Pin_Config.Pin_PullUp_PullDown = None;

    GPIO_Handle_t GpioButton;
    GpioButton.pGPIOx = GPIOA;
    GpioButton.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No0;
    GpioButton.GPIO_Pin_Config.Pin_Moder = Input_Mode;
    GpioButton.GPIO_Pin_Config.Pin_Moder = Interrupt_Falling_Edge;
    GpioButton.GPIO_Pin_Config.Pin_Speed = Medium_speed;
    // GpioButton.GPIO_Pin_Config.Pin_OutputType = Output_PushPull;
    GpioButton.GPIO_Pin_Config.Pin_PullUp_PullDown = None;

    // SYS_Config_Clock_Tree();

    GPIO_PeripheralClockControl(GPIOD,Enable);
    GPIO_PeripheralClockControl(GPIOA,Enable);
    GPIO_Initialize(&GpioLed);
    GPIO_Initialize(&GpioButton);
    NVIC_IRQInterruptConfig(IRQNumber_EXTI0,Enable);
    SysTick_Initialize(TICK_NUMBER_IN_1MS);

    while(1)
    {
        // if(GPIO_ReadFromInputPin(GPIOA,GPIO_Pin_No0) == High)
        // {
        //     delay();
        //     flag++;
        
        //     if(flag % 2 != 0)
        //     {
        //     GPIO_WriteFromOutputPin(GPIOD,GPIO_Pin_No14,Set);
        //     }
        //     else 
        //     {
        //     GPIO_WriteFromOutputPin(GPIOD,GPIO_Pin_No14,Reset);
        //     }
        // }
         if(flag % 2 != 0)
            {

            GPIO_ToggleFromOutputPin(GPIOD,GPIO_Pin_No14);
            Delay(100); /* Delay ms*/
            }
            else 
            {
            GPIO_WriteFromOutputPin(GPIOD,GPIO_Pin_No14,Reset);
            }

    }
    return 0;
}

void EXTI0_IRQHandler(void)
{
    GPIO_DetermineAlreadyInterrupt(GPIO_Pin_No0);
    flag++;
}