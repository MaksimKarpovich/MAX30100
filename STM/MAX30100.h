/**
  ******************************************************************************
  * @file    MAX30100.h
  * @brief   This file contains all the function prototypes for
  *          the MAX30100.c file
  ******************************************************************************
  */
#include "main.h"
#include "CalcHR_SpO2.h"

#define NUMBER_OF_SAMPLE_RATES 8

/**
  * @brief  Data update frequency
  */
typedef enum {
    RATE50HZ = 50,
    RATE100HZ = 100,
    RATE167HZ = 167,
    RATE200HZ = 200,
    RATE400HZ = 400,
    RATE600HZ = 600,
    RATE800HZ = 800,
    RATE1000HZ = 1000
} SampleRateUserValue;

/**
  * @brief  Describes the module working status
  */
typedef enum {
    MAX30100_OK,      ///<Module is working successfully
    MAX30100_ERROR    ///<Module is malfunctioning
} Max30100ControlStatus;

/**
  * @brief  Finger location status
  */
typedef enum {
    IS_FINGER_PLACED,
    IS_NOT_FINGER_PLACED
} FingerStatus;

/**
  * @brief  LED current
  */
typedef enum {
    CURRENT_0MA,        ///<LED current = 0 mA
    CURRENT_4_4MA,      ///<LED current = 4.4 mA
    CURRENT_7_6MA,      ///<LED current = 7.6 mA
    CURRENT_11MA,       ///<LED current = 11 mA
    CURRENT_14_2MA,     ///<LED current = 14.2 mA
    CURRENT_17_4MA,     ///<LED current = 17.4mA
    CURRENT_20_8MA,     ///<LED current = 20.8 mA
    CURRENT_24MA,       ///<LED current = 24 mA
    CURRENT_27_1MA,     ///<LED current = 27.1 mA
    CURRENT_30_6MA,     ///<LED current = 30.6 mA
    CURRENT_33_8MA,     ///<LED current = 33.8 mA
    CURRENT_37MA,       ///<LED current = 37 mA
    CURRENT_40_2MA,     ///<LED current = 40.2 mA
    CURRENT_43_6MA,     ///<LED current = 43.8 mA
    CURRENT_46_8MA,     ///<LED current = 46.8 mA
    CURRENT_50MA,       ///<LED current = 50 mA
} LedCurrentControl;

/**
  * @brief  Inerrupt status
  */
typedef enum {
    INTERRUPT_DISABLE,       ///<Interruption is disable
    INTERRUPT_ENABLE        ///<Interruption is enable
} InterruptStatus;

/**
  * @brief  Sensor working mode
  */
typedef enum {
    MAX_MODE_OFF = 0,           ///Sensor is off
    MAX_MODE_HR_ONLY = 2,       ///<Works by measuring only the heart rate
    MAX_MODE_HR_SPO2 = 3        ///<Works by measuring heart rate and SpO2
} MaxMode;

/**
  * @brief Pulse width and ADC resolution
  */
typedef enum {
    PULSE_WIDTH_200,        ///<Pulse width = 200µs,  ADC resolution = 13 bits
    PULSE_WIDTH_400,        ///<Pulse width = 400µs,  ADC resolution = 14 bits
    PULSE_WIDTH_800,        ///<Pulse width = 800µs,  ADC resolution = 15 bits
    PULSE_WIDTH_1600,       ///<Pulse width = 1600µs, ADC resolution = 16 bits
} AdcPulseWidth;

/**
  * @brief Initialization register status to set data update frequency
  */
typedef enum {
    INIT_RATE50HZ,
    INIT_RATE100HZ,
    INIT_RATE167HZ,
    INIT_RATE200HZ,
    INIT_RATE400HZ,
    INIT_RATE600HZ,
    INIT_RATE800HZ,
    INIT_RATE1000HZ
} SampleRateRegData;

/**
  * @brief MAX30100 init structure definition
  */
typedef struct {
    LedCurrentControl redLedCurrent;
    LedCurrentControl irLedCurrent;
    InterruptStatus allFifoFull;
    InterruptStatus temperatureReady;
    InterruptStatus heartReady;
    InterruptStatus spO2Ready;
    AdcPulseWidth ledPulseWidth;
    SampleRateRegData initSampleRate;
    MaxMode mode;
} Max30100Init;

/**
  * @brief  Initializes the sensor MAX30100.
  *
  * @note   This function set mode and power of LEDs, enable interrupts.
  *         Also check initialization success.
  * @return Status of initialization
  */
Max30100ControlStatus initializeMax30100(Max30100Init);

/**
  * @brief  Read samples from cycleBuff.
  *
  * @note   Read samples from cycleBuff to irBuff and redBuff respectively.
  * @param  irBuff address of infrared array, where the values are written
  * @param  redBuff address of red array, where the values are writing
  * @param  samplesNum Number of samples to writting
  * @return None
  */
void readLastSamples(uint16_t* irBuff, uint16_t* redBuff, uint16_t samplesNum);

/**
  * @brief  Read BUFFER_SIZE samples from MAX30100.
  *
  * @note   This function uses to read BUFFER_SIZE samples from sensor to buffer.
  * @return None
  */
void readDataMax30100(void);
