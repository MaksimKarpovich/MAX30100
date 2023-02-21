/**
  ******************************************************************************
  * @file    MAX30100.c
  * @brief   This file is used to work with heart rate sensor MAX30100
  ******************************************************************************
  */


#include "MAX30100.h"
#include "i2c.h"
#include <stdio.h>
#include <stdbool.h>
#include "BrightnessControl.h"

#define REGISTER_INT_STATUS  0x00  		///< Which interrupts are tripped
#define REGISTER_INT_ENABLE  0x01  		///< Which interrupts are active
#define REGISTER_FIFO_WR_PTR 0x02  		///< Where data is being written
#define REGISTER_OVRFLOW_CTR 0x03  		///< Number of lost samples
#define REGISTER_FIFO_RD_PTR 0x04  		///< Where to read from
#define REGISTER_FIFO_DATA   0x05  		///< Ouput data buffer
#define REGISTER_MODE_CONFIG 0x06  		///< Control register
#define REGISTER_SPO2_CONFIG 0x07  		///< Oximetry settings
#define REGISTER_LED_CONFIG  0x09  		///< Pulse width and power of LEDs
#define REGISTER_TEMP_INT    0x16  		///< Temperature integer
#define REGISTER_TEMP_FRAC   0x17  		///< Temperature fraction
#define REGISTER_REV_ID      0xFE  		///< Part revision
#define REGISTER_PART_ID     0xFF  		///< Part ID, normally 0x11

// Sensor Interrupt Status register and Interrupt Enable Flags
#define INT_A_FULL		(1<<7)		///< FIFO buffer almost full (1 sample left)
#define INT_TEMP_RDY	(1<<6)		///< Temperature measurement completed
#define INT_HR_RDY		(1<<5)		///< Heart rate measurement completed
#define INT_SPO2_RDY	(1<<4)		///< SpO2 measurement completed
#define INT_PWR_RDY		(1<<0)		///< Sensor ready to collect data after power reset

// Sensor mode flags
#define MODE_SHDN		(1<<7)		///< Mode Shutdown
#define MODE_RESET		(1<<6)		///< Soft sensor reset
#define MODE_TEMP_EN	(1<<3)		///< Start sensor temperature measurement

#define SPO2_CONFIG_CONSTANT 2
#define LED_FINGER_LIMIT 10000      ///<The minimum value accepted by the photodiode at which the finger is in front of the sensor
#define BYTE_OFFSET 8               ///<Number of bits in one byte
#define TWO_SAMPLES_BYTE_SIZE 4

FingerStatus fingrStatus = IS_NOT_FINGER_PLACED; ///<Current finger status

/**
  * @brief  Describes the current state of light intensity
  */
typedef struct {
    uint16_t ir;    ///< Light intensity of infrared LED
    uint16_t red;   ///< Light intensity of red LED
} tSample;

SampleRateUserValue sampleRateUserValue[NUMBER_OF_SAMPLE_RATES] = {RATE50HZ,
                                                                   RATE100HZ,
                                                                   RATE167HZ,
                                                                   RATE200HZ,
                                                                   RATE400HZ,
                                                                   RATE600HZ,
                                                                   RATE800HZ,
                                                                   RATE1000HZ
                                                                  };
SampleRateRegData sampleRateRegData = 0;

static tSample cycleBuff[BUFFER_SIZE] = {0, };  ///<Cyclic buffer of red and infrared raw values
static uint16_t writeIndex = 0;                 ///< Index showing where the writing pointer is currently located

Max30100ControlStatus initializeMax30100(Max30100Init initStruct)
{
    sampleRateRegData = initStruct.initSampleRate;

    struct LedCurrent {
        uint8_t irLed:4;
        uint8_t redLed:4;
    };

    struct IntConfig {
        uint8_t nothing:4;
        uint8_t spO2Ready:1;
        uint8_t heartReady:1;
        uint8_t temperaturReady:1;
        uint8_t fifoFull:1;
    };

    struct SpO2Config {
        uint8_t ledPulseWidth:2;
        uint8_t sampleRate:3;
        uint8_t constant:3;
    };

    struct SpO2Config spO2Config = {initStruct.ledPulseWidth,
               initStruct.initSampleRate,
               SPO2_CONFIG_CONSTANT
    };
    struct IntConfig intConfig = {0b0000,
               initStruct.spO2Ready,
               initStruct.heartReady,
               initStruct.temperatureReady,
               initStruct.allFifoFull
    };
    struct LedCurrent ledCurrent = {initStruct.irLedCurrent,
               initStruct.redLedCurrent
    };

    HAL_StatusTypeDef i2CStatus;

    uint8_t txmode[3] = {REGISTER_MODE_CONFIG,
                         initStruct.mode,
                         *((uint8_t*)&spO2Config)
                        },
                        txled[2] = {REGISTER_LED_CONFIG,
                                    *((uint8_t*)&ledCurrent)
                                   },
                                   txint[2] = {REGISTER_INT_ENABLE,
                                               *((uint8_t*)&intConfig)
                                              },
                                           txreset[2] = {REGISTER_MODE_CONFIG, MODE_RESET},
                                                   rxmode[2] = {0, },
                                                           rxled= 0,
                                                           rxint = 0,
                                                           rxreset = MODE_RESET;

    MX_I2C1_Init();

    while(HAL_GPIO_ReadPin(MAX_INT_GPIO_Port, MAX_INT_Pin) == GPIO_PIN_RESET) {}
    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txreset, sizeof(txreset));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txreset.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    while(HAL_GPIO_ReadPin(MAX_INT_GPIO_Port, MAX_INT_Pin) == GPIO_PIN_RESET) {}
    do {
        i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txreset, sizeof(txreset[0]));
        if(i2CStatus != HAL_OK)
        {
            printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
            printf("Error writing rxreset address.\r\n");
            return MAX30100_ERROR;
        }
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

        i2CStatus = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, &rxreset, sizeof(rxreset));
        if(i2CStatus != HAL_OK)
        {
            printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
            printf("Error reading rxreset.\r\n");
            return MAX30100_ERROR;
        }
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    } while(rxreset != 0);

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txled, sizeof(txled));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txled.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txmode, sizeof(txmode));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txmode.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txmode, sizeof(txmode[0]));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txmode address.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, rxmode, sizeof(rxmode));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error reding rxmode.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    if((txmode[1] != rxmode[0]))
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Read mode value does not match mode written value\r\n");
        printf("Writted value: %d\r\n", txmode[1]);
        printf("Readed value: %d\r\n", rxmode[0]);
        return MAX30100_ERROR;
    }

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txled, sizeof(txled));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txled.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txled, sizeof(txled[0]));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txled address.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, &rxled, sizeof(rxled));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error reading txled.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    if((txled[1] != rxled))
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Read led value does not match led written value\r\n");
        printf("Writted value: %d\r\n", txled[1]);
        printf("Readed value: %d\r\n", rxled);
        return MAX30100_ERROR;
    }

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txint, sizeof(txint));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txint.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, txint, sizeof(txint[0]));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error writing txint address.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    i2CStatus = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, &rxint, sizeof(rxint));
    if(i2CStatus != HAL_OK)
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Error reading txint.\r\n");
        return MAX30100_ERROR;
    }
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    if((txint[1] != rxint))
    {
        printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
        printf("Read interruption value does not match interruption written value\r\n");
        printf("Writted value: %d\r\n", txint[1]);
        printf("Readed value: %d\r\n", rxint);
        return MAX30100_ERROR;
    }

    bool haveInitError = (i2CStatus != HAL_OK) || (txmode[1] != rxmode[0]) || (txled[1] != rxled) || (txint[1] != rxint);

    if(!haveInitError)
        return MAX30100_OK;
    else
        return MAX30100_ERROR;
}

void readLastSamples(uint16_t *irBuff, uint16_t *redBuff, uint16_t samplesNum)
{
    for(uint16_t i = 0; i < samplesNum; i++)
    {
        if(i < (samplesNum - DIFFSTEP))
        {
            *(irBuff++) = cycleBuff[i + DIFFSTEP].ir;
            *(redBuff++) = cycleBuff[i + DIFFSTEP].red;
        }
    }

}

/**
  * @brief  Write one sample.
  *
  * @note   This function uses to write one sample to buffer.
  * @return None
  */
static void writeSample(tSample *sample)
{
    cycleBuff[writeIndex].ir = sample->ir;
    cycleBuff[writeIndex].red = sample->red;

    writeIndex++;
    if (writeIndex == BUFFER_SIZE)
        writeIndex = 0;
}

/**
  * @brief  Read one sample from MAX30100.
  *
  * @note   This function uses to read one sample from sensor to buffer.
  * @return None
  */
static void readSamples(void)
{
    uint8_t buff[TWO_SAMPLES_BYTE_SIZE] = {0, }, addres = REGISTER_FIFO_DATA;

    HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, &addres, sizeof(addres));
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
    HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS,(uint8_t *) buff, sizeof(buff));
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

    tSample newSample;
    newSample.ir = (buff[0] << BYTE_OFFSET) | buff[1];
    newSample.red = (buff[2] << BYTE_OFFSET) | buff[3];
    writeSample(&newSample);

    if(newSample.ir > LED_FINGER_LIMIT)
        fingrStatus = IS_FINGER_PLACED;
    else
        fingrStatus = IS_NOT_FINGER_PLACED;
}

void readDataMax30100(void)
{
    HAL_StatusTypeDef i2CStatus;
    uint8_t Status = 0,
            Address = REGISTER_INT_STATUS;

    for(uint16_t i = 0; i < (BUFFER_SIZE);)
    {
        i2CStatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, SLAVE_ADDRESS, &Address, sizeof(Address));
        if(i2CStatus == HAL_ERROR)
        {
            printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
            printf("Error writing address of interruptions statuses.\r\n");
        }
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

        i2CStatus = HAL_I2C_Master_Receive_DMA(&hi2c1, SLAVE_ADDRESS, &Status, sizeof(Status));
        if(i2CStatus == HAL_ERROR)
        {
            printf("\r\nFail: Funtcion: %s, Line: %i.\r\n", __FUNCTION__, __LINE__);
            printf("Error reading address of interruptions statuses.\r\n");
        }
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
        if (Status & (INT_A_FULL | INT_HR_RDY))
        {
            readSamples();
            i++;
        }
    }
}
