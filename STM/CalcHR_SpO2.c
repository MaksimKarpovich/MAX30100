/**
  ******************************************************************************
  * @file    CalcHR_SpO2.c
  * @brief   This file is used to process values from heart rate sensor MAX30100
  ******************************************************************************
  */


#include "CalcHR_SpO2.h"
#include "MAX30100.h"
#include "BrightnessControl.h"

#define FILTERWIDTH 30              ///<Width of moving average filter
#define REDUCTION_ARRAY 23          ///<Array reduction value
#define REDUCTION_READ 15           ///<Read array reduction value
#define NUM_SECONDS_IN_MINUTE 60
#define NUM_WAVES 2                 ///<Number of analyzed heart beat waves
#define START_INDEX_DEVIDE_СOEF 5   ///<Coefficient to take values starting from the established ones
#define DIFF_RANGE 50000            ///<Range between top and bottom

extern SampleRateRegData sampleRateRegData;
extern SampleRateUserValue sampleRateUserValue[NUMBER_OF_SAMPLE_RATES];

static uint16_t irMax,              ///<Maximum value of the raw value of the infrared LED for normalization
       irMin;                       ///<Minimum value of the raw value of the infrared LED for normalization
static uint16_t redMax,             ///<Maximum value of the raw value of the red LED for normalization
       redMin;                      ///<Minimum value of the raw value of the red LED for normalization
static uint16_t syncIndex = 0;      ///<Index for synchronizing extremes

static uint16_t sampleCount = 0;    ///<Sample counter during heart beat search
static uint16_t previousSample = 0; ///<Value of previous infrared LED value during heart beat search

static float r = 0;                 ///<Last time measured R coefficient value
static float spO2 = 0;              ///<Last time measured SpO2 value
static float heartRate = 0;         ///<Last time measured heart rate

static uint16_t irInputBuff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };             ///<Raw infrared LED value to normalizing
static uint16_t irStage1Buff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };            ///<Normalized infrared LED value to filtering
static uint16_t irStage2Buff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };            ///<Filtered infrared LED value to differentiation
static uint16_t irStage3Buff[BUFFER_SIZE - REDUCTION_ARRAY - DIFFSTEP] = {0, }; ///<Differentiated infrared LED value to heart beat search
static uint16_t redInputBuff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };            ///<Raw red LED value to normalizing
static uint16_t redStage1Buff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };           ///<Normalized red LED value to filtering
static uint16_t redStage2Buff[BUFFER_SIZE - REDUCTION_ARRAY] = {0, };           ///<Filtered red LED value to SpO2 calculated

/**
  * @brief  Find the minimum and maximum of the array.
  *
  * @note   This function uses to find global minimum and maximum of the array between index1 and index2.
  * @param  buff address of investigated array
  * @param  index1 starting indexv
  * @param  index2 ending index
  * @param  max the address where the maximum value will be written
  * @param  min the address where the minimum value will be written
  * @return The difference between the maximum and minimum values of the input array
  */
static uint16_t calculateMaxMinIndex(uint16_t* buff, uint16_t index1, uint16_t index2, uint16_t* max, uint16_t* min)
{
    *max = *min = *buff;
    uint16_t index = index1;
    while (index <= index2)
    {
        if (buff[index] < *min)
            *min = buff[index];
        if (buff[index] > *max)
            *max = buff[index];
        index++;
    }

    return *max - *min;
}

/**
  * @brief  Find the minimum and maximum of the array.
  *
  * @note   This function uses to find global minimum and maximum of the array.
  * @param  buff address of investigated array
  * @param  max the address where the maximum value will be written
  * @param  min the address where the minimum value will be written
  * @return The difference between the maximum and minimum values of the input array
  */
static uint16_t calculateMaxMin(uint16_t* buff, uint16_t samples, uint16_t* max, uint16_t* min)
{
    calculateMaxMinIndex(buff, 0, samples - 1, max, min);
    const  uint16_t sensitivity = 500;
    if (*max - *min < sensitivity)
        *max = *min + sensitivity;

    return *max - *min;
}

/**
  * @brief  Scales the array.
  *
  * @note   This function scales an array to a range [0, 65535].
  * @param  buffIn address of input array
  * @param  buffOut address of output array
  * @param  samples size of input array
  * @param  max maximum value of input array
  * @param  min minimum value of input array
  * @return None
  */
static void normaliseBuff(uint16_t* buffIn, uint16_t* buffOut, uint16_t samples, uint16_t max, uint16_t min)
{
    uint16_t delta = max - min;
    while (samples--)
    {
        uint32_t temp = *buffIn - min;
        temp = temp * 65535 / delta;
        *buffOut = (uint16_t) temp;
        buffIn++;
        buffOut++;
    }
}

/**
  * @brief  Moving average filter.
  *
  * @note   This function filter input array to remove noise.
  * @param  buffIn address of input array
  * @param  buffOut address of output array
  * @param  samples size of input array
  * @param  window filter width
  * @return None
  */
static void filterBuff(uint16_t* buffIn, uint16_t* buffOut, uint16_t samples, uint8_t window)
{
    if ((window % 2) == 0)
        window++;

    uint8_t hw = (window - 1) / 2;
    buffOut[0] = buffIn[0];

    for (uint16_t i = 1; i < samples; i++)
    {
        uint32_t sum = 0;
        uint16_t k1, k2;
        uint8_t samplesCounter;

        if (i < hw)
        {
            k1 = 0;
            k2 = 2 * i;
            samplesCounter = k2 + 1;
        }
        else if ((i + hw) > (samples - 1))
        {
            k1 = i - samples + i + 1;
            k2 = samples - 1;
            samplesCounter = k2 - k1 + 1;
        }
        else
        {
            k1 = i - hw;
            k2 = i + hw;
            samplesCounter = window;
        }

        for (uint16_t j = k1; j <= k2; j++)
            sum += buffIn[j];

        buffOut[i] = sum / samplesCounter;
    }
}

/**
  * @brief  Finds the derivative.
  *
  * @note   This function finds numerical difference using zero overflow.
  * @param  buffIn address of input array
  * @param  buffOut address of output array
  * @param  samples size of input array
  * @param  step value proportional to differentiation time
  * @param  offset coefficient using to zero overflow
  * @return None
  */
static void calculateDifferentialBuff(uint16_t* buffIn, uint16_t* buffOut, uint16_t samples, uint8_t step, uint16_t offset)
{
    for (uint16_t i = 1; i < samples; i++)
    {
        if (i >= step)
            buffOut[i - step] = (buffIn[i]) + offset - (buffIn[i - step]);
    }
}

/**
  * @brief  Finds the heart beat.
  *
  * @note   This function finds indices of heart beat.
  *         Using indices of heart beat find heart rate.
  * @param  buff address of array
  * @param  samples size of array
  * @param  beatIndex1 address where placed index of first heart beat
  * @param  beatIndex2 address where placed index of second heart beat
  * @return Calculated heart rate (beats per minute)
  */
static float getBeat(uint16_t* buff, uint16_t samples, uint16_t* beatIndex1, uint16_t* beatIndex2)
{
    uint8_t firstBeat = 1;
    uint16_t startIndex = sampleRateUserValue[sampleRateRegData] / START_INDEX_DEVIDE_СOEF;
    previousSample = buff[startIndex++];
    *beatIndex1 = *beatIndex2 = 0;

    for (uint16_t i = startIndex; i < (samples); i++)
    {
        uint16_t newValue = buff[i];
        if ((newValue < previousSample) && (previousSample - newValue > DIFF_RANGE))
        {

            if (firstBeat)
            {
                firstBeat = 0;
                sampleCount = 0;
                syncIndex = i;
                *beatIndex1 = i;
            }
            else
            {
                if (sampleCount)
                {
                    *beatIndex2 = i;
                    return (NUM_SECONDS_IN_MINUTE * sampleRateUserValue[sampleRateRegData]) /
                           (NUM_WAVES * sampleCount);
                }
            }
        }
        sampleCount++;
        previousSample = newValue;
    }

    syncIndex = 0;
    return 0;
}

float calculateMax30100(void)
{
    uint16_t samples = BUFFER_SIZE - REDUCTION_READ;
    const uint8_t deltaConst = 3;
    readLastSamples(irInputBuff, redInputBuff, samples);

    samples = samples - DIFFSTEP;

    uint16_t irDelta = calculateMaxMin(irInputBuff, samples, &irMax, &irMin);
    normaliseBuff(irInputBuff, irStage1Buff, samples, irMax, irMin);
    filterBuff(irStage1Buff, irStage2Buff, samples, FILTERWIDTH);
    calculateDifferentialBuff(irStage2Buff, irStage3Buff, samples, DIFFSTEP, irDelta * deltaConst);

    calculateMaxMin(redInputBuff, samples, &redMax, &redMin);
    normaliseBuff(redInputBuff, redStage1Buff, samples, redMax, redMin);
    filterBuff(redStage1Buff, redStage2Buff, samples, FILTERWIDTH);

    uint16_t index1, index2;
    heartRate = getBeat(irStage3Buff, samples, &index1, &index2);

    uint16_t irMin2, irMax2, redMin2, redMax2;
    irDelta = calculateMaxMinIndex(irInputBuff, index1, index2, &irMax2, &irMin2);
    uint16_t redDelta = calculateMaxMinIndex(redInputBuff, index1, index2, &redMax2, &redMin2);

    const uint8_t rСoefficients[2] = {104, 17};
    if ((redMin2 != 0.0) && (irDelta != 0.0))
    {
        r = redDelta * irMin2;
        r /= redMin2;
        r /= irDelta;
        spO2 = rСoefficients[0] - (rСoefficients[1] * r);
    }

    return getHeartRate();
}

float getHeartRate(void)
{
    const uint8_t lowLimit = 40, highLimit = 150;
    if ((heartRate < lowLimit) || (heartRate > highLimit))
        return (float)0;
    else
        return heartRate;
}

float getR(void)
{
    return r;
}

float getSpO2(void)
{
    const uint8_t lowLimit = 50, highLimit = 110;
    if ((spO2 < lowLimit) || (spO2 > highLimit))
        return (float)0;
    else
        return spO2;
}
