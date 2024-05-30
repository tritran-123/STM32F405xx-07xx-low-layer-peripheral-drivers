 #include "stm32f407vg_Interrupt.h"
  /* Interrupt function */
void NVIC_SetPriority (uint8_t IRQNumber, uint8_t Priority)
 {
    uint8_t IPR = IRQNumber / 4; /* determin which IPR register in Interrupt Priority Registers */
    uint8_t PRI_4bit = ((IRQNumber %4)*8 + 4); /* assign priority 0x0-0xf in last 4bit in PRI coressponding IRQNumber */
    (*(NVIC_IPR + IPR)) |= (Priority << PRI_4bit); 
 }

void NVIC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{
    uint8_t temp1 = IRQNumber / 32; /* determine which register ISER(0-7)*/
    uint8_t temp2 = IRQNumber % 32; /* determine which bit need to set value for it interrupt line */
    if (EnOrDis == Enable)
    {
        NVIC_ISER->ISER[temp1] |= (1 << temp2); /* 1 = enable interrupt.*/
    }
    else
    {
        NVIC_ICER->ICER[temp1] |= (1 << temp2); /*  1 = disable interrupt.*/
    }
}

/* System tick Timer function */
static uint32_t TimeDelay = 0; /* Declare globalvairiable to decrease time 1unit every 1 ms */
void SysTick_Initialize(uint32_t nTick)
{
    /* Disabled Counter */
    SysTick->CSR &= ~(1U << 0);

    /* Set Reload Register (24bit)*/
    SysTick->RVR |= nTick;

    /* Reset Current Tick Value Counter */
    SysTick->CVR = 0;

    /* Select Proccessor Clock */
    SysTick->CSR |= (1 << 2);

    /* Enabled SysTick Interrupt */
    SysTick->CSR |= (1 << 1);

    /* Enabled Counter */
    SysTick->CSR |= (1 << 0);
}

void SysTick_Handler(void)
{
    if (TimeDelay > 0)
    {
        TimeDelay--; /* with 1ms will have interrupt occurs */
    }
}

void Delay(uint32_t nTime)
{
    TimeDelay = nTime; /* assign g_nTimeDelay value to decrement */
    while (TimeDelay != 0)
    {
    }
}