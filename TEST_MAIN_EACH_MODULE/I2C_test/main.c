#include "..\STM32F407VG_Library\stm32f407vg_gpio_api.h"
#include "..\STM32F407VG_Library\stm32f407vg_i2c_driver.h"
// #include "..\STM32F407VG_Library\stm32f407vg_ClockConfig.h"
// #include "..\STM32F407VG_Library\stm32f407vg_Interrupt.h"
#include <string.h>

#define Slave_Addr 0x68
#define My_Addr 0x61
void I2C_Config(void);
void GPIO_Config(void);

I2C_Handle_t I2CHandle;
uint8_t data[] = "Hello Tri Tran\n";
uint8_t datareceive[20] = {0};

int main(void)
{
    I2C_Config();
    GPIO_Config();
    I2C_PeripheralControl(I2C1,Enable);
    while(1)
    {
        I2C_MasterReceiveData(&I2CHandle,datareceive,10,Slave_Addr,I2C_ContinuesCircuitData_Disable);
        while (1);
        
    }
}
void I2C_Config(void)
{
    I2CHandle.I2C_Config.I2C_AckControl = I2C_AckControl_Enable;
    I2CHandle.I2C_Config.I2C_SCLSPEED = I2C_SCLSPEED_SM;
    I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FMDutyCycle_2;
    I2CHandle.I2C_Config.I2C_DeviceAddress = My_Addr;
    I2CHandle.pI2Cx = I2C1;
    I2C_Initialize(&I2CHandle);
}
void GPIO_Config(void)
{
    GPIO_Handle_t GpioI2C;
    GpioI2C.pGPIOx = GPIOB;
    GpioI2C.GPIO_Pin_Config.Pin_Moder = Alt_Func_Mode;
    GpioI2C.GPIO_Pin_Config.Pin_Alternate_Func = AF4;
    GpioI2C.GPIO_Pin_Config.Pin_Speed = High_speed;
    GpioI2C.GPIO_Pin_Config.Pin_OutputType = Output_OpenDrain;
    GpioI2C.GPIO_Pin_Config.Pin_PullUp_PullDown = Pull_up;
    /*  PB6 : I2C_SCL */
    GpioI2C.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No6;
    GPIO_Initialize(&GpioI2C);
    /*  PB7 : I2C_SDA */
    GpioI2C.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No7;
    GPIO_Initialize(&GpioI2C);
}
