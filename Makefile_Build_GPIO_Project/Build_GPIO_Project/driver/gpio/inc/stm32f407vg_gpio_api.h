
#ifndef STM32F407VG_GPIO_API_H
#define STM32F407VG_GPIO_API_H

#include "stm32f407vg.h"
/* define information about each bit field */
#define GPIOAEN 0
#define GPIOBEN 1
#define GPIOCEN 2
#define GPIODEN 3
#define GPIOEEN 4
#define GPIOFEN 5
#define GPIOGEN 6
#define GPIOHEN 7
#define GPIOIEN 8
#define GPIOJEN 9
#define GPIOKEN 10

/* Function enable clock gpio for each portx */
#define GPIOA_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOAEN))
#define GPIOB_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOBEN))
#define GPIOC_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOCEN))
#define GPIOD_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIODEN))
#define GPIOE_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOEEN))
#define GPIOF_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOFEN))
#define GPIOG_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOGEN))
#define GPIOH_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOHEN))
#define GPIOI_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOIEN))
#define GPIOJ_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOJEN))
#define GPIOK_Enable_Clock() ((RCC->AHB1ENR) |= (1 << GPIOKEN))

/* Function disable clock gpio for each portx */
#define GPIOA_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOAEN))
#define GPIOB_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOBEN))
#define GPIOC_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOCEN))
#define GPIOD_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIODEN))
#define GPIOE_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOEEN))
#define GPIOF_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOFEN))
#define GPIOG_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOGEN))
#define GPIOH_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOHEN))
#define GPIOI_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOIEN))
#define GPIOJ_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOJEN))
#define GPIOK_Disable_Clock() ((RCC->AHB1ENR) &= ~(1U << GPIOKEN))

#define Input_Mode 0
#define GPOutput_Mode 1
#define Alt_Func_Mode 2
#define Analog_Mode 3
#define Interrupt_Falling_Edge 4
#define Interrupt_Rising_Edge 5
#define Interrupt_FallingRising_Edge 6

#define Output_PushPull 0
#define Output_OpenDrain 1

#define Low_speed 0
#define Medium_speed 1
#define High_speed 2
#define Very_high_speed 3

#define None 0
#define Pull_up 1
#define Pull_down 2
#define Reserved 3

#define GPIO_Pin_No0    0
#define GPIO_Pin_No1    1
#define GPIO_Pin_No2    2
#define GPIO_Pin_No3    3
#define GPIO_Pin_No4    4
#define GPIO_Pin_No5    5
#define GPIO_Pin_No6    6
#define GPIO_Pin_No7    7
#define GPIO_Pin_No8    8
#define GPIO_Pin_No9    9
#define GPIO_Pin_No10   10
#define GPIO_Pin_No11   11
#define GPIO_Pin_No12   12
#define GPIO_Pin_No13   13
#define GPIO_Pin_No14   14
#define GPIO_Pin_No15   15

#define AF0    0
#define AF1    1
#define AF2    2
#define AF3    3
#define AF4    4
#define AF5    5
#define AF6    6
#define AF7    7
#define AF8    8
#define AF9    9
#define AF10    10
#define AF11    11
#define AF12    12
#define AF13    13
#define AF14    14
#define AF15    15

/* define struct to handle function config gpio */
typedef struct
{
    uint8_t Pin_Number;
    uint8_t Pin_Moder;
    uint8_t Pin_OutputType;
    uint8_t Pin_Speed;
    uint8_t Pin_PullUp_PullDown;
    uint8_t Pin_Alternate_Func;
} GPIO_Pin_config_t;

typedef struct
{
    GPIO_Pin_config_t GPIO_Pin_Config;
    RegDef_GPIO_Portx_t *pGPIOx;
} GPIO_Handle_t;





/* prototype function in file stm32f407vg_gpio_driver.c*/
void GPIO_PeripheralClockControl(RegDef_GPIO_Portx_t *pGPIOx, uint8_t EnOrDi);
void GPIO_Initialize(GPIO_Handle_t *pGPIOxHandle);
uint8_t GPIO_ReadFromInputPin(RegDef_GPIO_Portx_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(RegDef_GPIO_Portx_t *pGPIOx);
void GPIO_WriteFromOutputPin(RegDef_GPIO_Portx_t *pGPIOx, uint8_t PinNumber,uint8_t SetOrRst);
void GPIO_WriteFromOutputPort(RegDef_GPIO_Portx_t *pGPIOx,uint16_t ValueWrite);
void GPIO_ToggleFromOutputPin(RegDef_GPIO_Portx_t *pGPIOx,uint8_t PinNumber);
/* interrupt function */
void GPIO_DetermineAlreadyInterrupt(uint8_t IRQNumber); /* This function use for interrupt in GPIO peripheral - clear flag interrupt to wait next interrupt*/



#endif

