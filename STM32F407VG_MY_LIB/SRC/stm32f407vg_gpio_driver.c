#include "stm32f407vg_gpio_api.h"
#include "stm32f407vg_Interrupt.h"

void GPIO_PeripheralClockControl(RegDef_GPIO_Portx_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == Enable)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_Enable_Clock();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_Enable_Clock();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_Enable_Clock();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_Enable_Clock();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_Enable_Clock();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_Enable_Clock();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_Enable_Clock();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_Enable_Clock();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_Enable_Clock();
        }
        else if (pGPIOx == GPIOJ)
        {
            GPIOJ_Enable_Clock();
        }
        else if (pGPIOx == GPIOK)
        {
            GPIOK_Enable_Clock();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_Disable_Clock();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_Disable_Clock();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_Disable_Clock();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_Disable_Clock();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_Disable_Clock();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_Disable_Clock();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_Disable_Clock();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_Disable_Clock();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_Disable_Clock();
        }
        else if (pGPIOx == GPIOJ)
        {
            GPIOJ_Disable_Clock();
        }
        else if (pGPIOx == GPIOK)
        {
            GPIOK_Disable_Clock();
        }
    }
}

void GPIO_Initialize(GPIO_Handle_t *pGPIOxHandle)
{   

    GPIO_PeripheralClockControl(pGPIOxHandle->pGPIOx, Enable);

    _vo uint32_t temp = 0;
    /* Config moder */
    /* Common mode*/
    if (pGPIOxHandle->GPIO_Pin_Config.Pin_Moder <= Analog_Mode)
    {
        temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_Moder) << (2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* shift left 2bit moder and assign to temp */
        pGPIOxHandle->pGPIOx->MODER &= ~(0x3U << 2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));              /* clear 2bit need to set in moder register */
        pGPIOxHandle->pGPIOx->MODER |= temp;                                                                  /* assign value */
    }
    /* Interrupt mode*/
    else
    { 
        /* Enable clock for module SYSCFG */
        SYSCFG_Enable_Clock();
        /* Enable value in Register SYSCFG_EXTICR(1-4) for corresponds 10 port */
        uint8_t temp1 = (pGPIOxHandle->GPIO_Pin_Config.Pin_Number) / 4;                                    /* determine which register for coresspends pin (EX : Pin(0-3) in EXTICR[0]...Pin(12-15) in EXTICR[3])*/
        uint8_t temp2 = (pGPIOxHandle->GPIO_Pin_Config.Pin_Number) % 4;                                    /* determine which bit need to assign value 1pin-(4bit) */
        uint8_t temp3 = Corresponds_Value_For_Each_Port(pGPIOxHandle->pGPIOx);
        SYSCFG->EXTICR[temp1] |= (temp3 << (temp2 * 4)); /* assign value(port) in 4bit coresspon for each pin
        (EX : PTB4 => (Corresponds_Value_For_Each_Port = 0x0001) and pin4 in Register SYSCFG[temp1 = 4 /4 =1]) and in field (temp2 = 4%4 *4  = 0 = bit (0-1-2-3))*/
        
        /*  Enable EXTI_IMR 1: Interrupt request from line x is not masked */
        EXTI->IMR |= (1 << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));
        
        /* Enable trigger status (falling -rising -falling rising ) interrupt */
        if (pGPIOxHandle->GPIO_Pin_Config.Pin_Moder == Interrupt_Falling_Edge)
        {
            EXTI->RTSR &= ~(1U << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* if use falling -> turn of rising */
            EXTI->FTSR |= (1 << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));  /* falling trigger enabled*/
        }
        else if (pGPIOxHandle->GPIO_Pin_Config.Pin_Moder == Interrupt_Rising_Edge)
        {
            EXTI->FTSR &= ~(1U << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* if use rising -> turn of falling trigger */
            EXTI->RTSR |= (1 << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));  /* rising trigger enabled*/
        }
        else if (pGPIOxHandle->GPIO_Pin_Config.Pin_Moder == Interrupt_FallingRising_Edge)
        {
            EXTI->RTSR |= (1 << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* rising trigger enabled*/
            EXTI->FTSR |= (1 << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* rising trigger enabled*/
        }

       
    }
    /* config Speed */
    temp = 0;
    temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_Speed) << (2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* shift left 2bit speed and assign to temp */
    pGPIOxHandle->pGPIOx->OSPEEDR &= ~(0x3U << 2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));            /* clear 2bit need to set in speed register */
    pGPIOxHandle->pGPIOx->OSPEEDR |= temp;                                                                /* assign value */
    /* Config OutputType*/
    temp = 0;
    temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_OutputType) << ((pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* shift left 1bit output-type (total write in 15bit) and assign to temp */
    pGPIOxHandle->pGPIOx->OTYPER &= ~(0x1U << (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));                  /* clear 1bit need to set in OType register */
    pGPIOxHandle->pGPIOx->OTYPER |= temp;                                                                  /* assign value */
    /* Config PUPD*/
    temp = 0;
    temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_PullUp_PullDown) << (2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* shift left 2bit PUPD and assign to temp */
    pGPIOxHandle->pGPIOx->PUPDR &= ~(0x3U << 2 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));                        /* clear 2bit need to set in PUPD register */
    pGPIOxHandle->pGPIOx->PUPDR |= temp;                                                                            /* assign value */
    /* Config Alternate_func*/
    if (pGPIOxHandle->GPIO_Pin_Config.Pin_Number <= 7)
    {
        temp = 0;
        temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_Alternate_Func) << (4 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number)); /* shift left 4bit altfunc for pin 0-7 and assign to temp */
        pGPIOxHandle->pGPIOx->AFRL &= ~(0x3U << 4 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));                        /* clear 4bit need to set in altfunc register */
        pGPIOxHandle->pGPIOx->AFRL |= temp;                                                                            /* assign value */
    }
    else
    {
        temp = 0;
        temp = (pGPIOxHandle->GPIO_Pin_Config.Pin_Alternate_Func) << (4 * ((pGPIOxHandle->GPIO_Pin_Config.Pin_Number) - 8)); /* shift left 4bit altfunc for pin 8-15 and assign to temp */
        pGPIOxHandle->pGPIOx->AFRH &= ~(0x3U << 4 * (pGPIOxHandle->GPIO_Pin_Config.Pin_Number));                              /* clear 4bit need to set in altfunc register */
        pGPIOxHandle->pGPIOx->AFRH |= temp;                                                                                  /* assign value */
    }
}

uint8_t GPIO_ReadFromInputPin(RegDef_GPIO_Portx_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t ReturnValue = 0;
    uint32_t Mask = 0x00000001;
    ReturnValue = (uint8_t)((pGPIOx->IDR >> PinNumber) & Mask);
    return ReturnValue;
}

uint16_t GPIO_ReadFromInputPort(RegDef_GPIO_Portx_t *pGPIOx)
{
    uint16_t ReturnValue = (uint16_t)(pGPIOx->IDR);
    return ReturnValue;
}

void GPIO_WriteFromOutputPin(RegDef_GPIO_Portx_t *pGPIOx, uint8_t PinNumber, uint8_t SetOrRst)
{
    if (SetOrRst == Set)
    {
        pGPIOx->ODR |= (Set << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1U << PinNumber);
    }
}

void GPIO_WriteFromOutputPort(RegDef_GPIO_Portx_t *pGPIOx, uint16_t ValueWrite)
{
    pGPIOx->ODR = (uint32_t)ValueWrite;
}

void GPIO_ToggleFromOutputPin(RegDef_GPIO_Portx_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (Set << PinNumber);
}

/* This function use for interrupt in GPIO peripheral - clear flag interrupt to wait next interrupt*/
void GPIO_DetermineAlreadyInterrupt(uint8_t PinNumber)
{
    /* check  1: selected trigger request occurred */
    if(EXTI->PR & (1 << PinNumber)) 
    {
        EXTI->PR |= (1 << PinNumber); /* This bit is set when the selected edge event arrives on the external interrupt line. 
      This bit is cleared by programming it to ‘1’. */
    }
}

