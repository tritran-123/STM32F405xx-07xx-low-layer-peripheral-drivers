#include "..\STM32F407VG_Library\stm32f407vg.h"
#include "..\STM32F407VG_Library\stm32f407vg_gpio_api.h"
#include "..\STM32F407VG_Library\stm32f407vg_spi_driver.h"
#include "..\STM32F407VG_Library\stm32f407vg_ClockConfig.h"

/*
    PB12 :SPI2_NSS
    PB13 :SPI2_SCK
    PB14 :SPI2_MISO
    PB15 :SPI2_MOSI
*/
void SPI_Config_Master(void);
void GPIO_Config(void);
void Initalize_Slave_8DigitLed_MAX7219(void);
int main(void)
{
    // uint16_t DataTurnFullLed[5] = {0x09FF,0x0A06,0x0B07,0x0F00,0x0C01};
    // uint8_t DataTurnFullLed[16] = {0x00,0x00,0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07, 0x08, 0x08};
    uint8_t DataTurnFullLed[16] = {0x00,0x00,0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04};

    // uint8_t DataSetUpLed[15] = {0x00,0x00,0x09,0x0F, 0x0A, 0x06, 0x0B, 0x07, 0x0F, 0x00, 0x0C, 0x01};
    uint8_t DataSetUpLed[15] = {0x00,0x00,0xFF,0x09, 0x06, 0x0A, 0x07, 0x0B, 0x00, 0x0F, 0x00, 0x0C};

    uint8_t one0x[2] = {0x01,0x02};
    // SYS_Config_Clock_Tree();
    SPI_Config_Master();
    GPIO_Config();
    // SPI_SSIEnable(SPI2,Enable);
    SPI_SSOEnable(SPI2, Enable);
    // GPIO_WriteFromOutputPin(GPIOB,GPIO_Pin_No12,Set);
    /*enable module spi */
    SPI_PeripheralEnable(SPI2, Enable);
    /* send data to init slave */
    // GPIO_WriteFromOutputPin(GPIOB,GPIO_Pin_No12,Reset);
    // GPIO_WriteFromOutputPin(GPIOB,GPIO_Pin_No12,Reset);
    // Initalize_Slave_8DigitLed_MAX7219();
    // GPIO_WriteFromOutputPin(GPIOB,GPIO_Pin_No12,Set);
    // SPI_SendData(SPI2, DataSetUpLed, 10);
    // SPI_Send16bit(SPI2, 0x0000);
    // SPI_Send16bit(SPI2, 0x09FF);
    // SPI_Send16bit(SPI2, 0x0A0E);
    // SPI_Send16bit(SPI2, 0x0B07);
    // SPI_Send16bit(SPI2, 0x0F00);
    // SPI_Send16bit(SPI2, 0x0C01);

    SPI_SendData(SPI2,DataSetUpLed,12);
    // while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

    SPI_PeripheralEnable(SPI2, Disable);

    while (1)
    {
        SPI_PeripheralEnable(SPI2, Enable);
         SPI_SendData(SPI2, DataTurnFullLed, 10);
        // SPI_Send16bit(SPI2, 0x0101);
        // SPI_Send16bit(SPI2, 0x0202);
        // SPI_Send16bit(SPI2, 0x0303);
        // SPI_Send16bit(SPI2, 0x0404);
        // SPI_Send16bit(SPI2, 0x0505);
        // SPI_Send16bit(SPI2, 0x0606);
        // SPI_Send16bit(SPI2, 0x0707);
        // SPI_Send16bit(SPI2, 0x0808);
        // SPI_Send16bit(SPI2, 0x0808);
    //SPI_SendData(SPI2,DataTurnFullLed,2);


        // while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
        SPI_PeripheralEnable(SPI2, Disable);
        while (1)
            ;
    }
    return 0;
}

void SPI_Config_Master(void)
{
    SPI_Handle_t SPI2handle;
    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPI_Config.SPI_BusConfig = SPI_BusConfig_HalfDuplex_TXONLY;
    SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DeviceMode_Master;
    SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SclkSpeed_Div16;
    SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_16bit;
    SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_FirstClock;
    SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LowIdle;
    SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DIS;
    SPI_Initialize(&SPI2handle);
}

void GPIO_Config(void)
{
    GPIO_Handle_t GpioSPI2;
    GpioSPI2.pGPIOx = GPIOB;
    GpioSPI2.GPIO_Pin_Config.Pin_Moder = Alt_Func_Mode;
    GpioSPI2.GPIO_Pin_Config.Pin_Alternate_Func = AF5;
    GpioSPI2.GPIO_Pin_Config.Pin_Speed = High_speed;
    GpioSPI2.GPIO_Pin_Config.Pin_OutputType = Output_PushPull;
    GpioSPI2.GPIO_Pin_Config.Pin_PullUp_PullDown = None;

    GPIO_PeripheralClockControl(GPIOB, Enable);

    /* SCLK PB13 */
    GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No13;
    GPIO_Initialize(&GpioSPI2);
    // /* MISO PB14 */
    GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No14;
    GPIO_Initialize(&GpioSPI2);
    /* MOSI PB15 */
    GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No15;
    GPIO_Initialize(&GpioSPI2);
    /* NSS PB12 */
    GpioSPI2.GPIO_Pin_Config.Pin_Number = GPIO_Pin_No12;
    // GpioSPI2.GPIO_Pin_Config.Pin_Moder = GPOutput_Mode;
    GPIO_Initialize(&GpioSPI2);
}

// void Initalize_Slave_8DigitLed_MAX7219(void)
// {

//     // uint16_t CodeInitSlave[] ={0x09FF,0x0A06,0x0B07,0x0F00,0x0C01};
//     // uint8_t CodeInitSlave[] ={0x09,0xFF,0x0A,0x06,0x0B,0x07,0x0F,0x00,0x0C,0x01};

//     /* MSB */
//     /* Decode-Mode Register Examples (Address (Hex) = 0xX9) */
//     /* Code B decode for digits 7–0 : 0xff */
//     // LPSPI_TransmitData(0x09FF);

//     // /* Intensity Register Format (Address (Hex) = 0xXA) */
//     // /* MAX7219(3/32) : 0xX8 */
//     // LPSPI_TransmitData(0x0A00);

//     // /* Scan-Limit Register Format (Address (Hex) = 0xXB) */
//     // /* Display digits 0 1 2 3 4 5 6 7 : 0xX7 */
//     // LPSPI_TransmitData(0x0B07);

//     // /*Display-Test Register Format (Address (Hex) = 0xXF)*/
//     // /* Normal Operation 0xX0 */
//     // LPSPI_TransmitData(0x0F00);

//     // /* Shutdown Register Format (Address (Hex) = 0xXC) */
//     // /* Normal Operation : 0xX1 */
//     // LPSPI_TransmitData(0x0C01);

//     SPI_SendData(SPI2,0x09FF);
//     SPI_SendData(SPI2,0x0A06);
//     SPI_SendData(SPI2,0x0B07);
//     SPI_SendData(SPI2,0x0F00);
//     SPI_SendData(SPI2,0x0C01);

// }
