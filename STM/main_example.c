/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

#include "MAX30100.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();

    Max30100Init max30100InitStruct = {0};

    max30100InitStruct.redLedCurrent = CURRENT_24MA;
    max30100InitStruct.irLedCurrent = CURRENT_24MA;
    max30100InitStruct.allFifoFull = INTERRUPT_ENABLE;
    max30100InitStruct.temperatureReady = INTERRUPT_ENABLE;
    max30100InitStruct.heartReady = INTERRUPT_ENABLE;
    max30100InitStruct.spO2Ready = INTERRUPT_ENABLE;
    max30100InitStruct.ledPulseWidth = PULSE_WIDTH_1600;
    max30100InitStruct.initSampleRate = INIT_RATE200HZ;
    max30100InitStruct.mode = MAX_MODE_HR_SPO2;

    Max30100ControlStatus maxInitStatus = initializeMax30100(max30100InitStruct);

    printf("Start of working.\r\n");
    while (1)
    {        
        readDataMax30100();
        calculateMax30100();
        
        if(getHeartRate() != 0) {
            printf("Heart rate: %d\r\n", (uint16_t)getHeartRate());
            printf("SpO2: %d%%\r\n", (uint16_t)getSpO2());
    	}
    }
}

