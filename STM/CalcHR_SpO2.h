/**
  ******************************************************************************
  * @file    CalcHR_SpO2.h
  * @brief   This file contains all the function prototypes for
  *          the CalcHR_SpO2.c file
  ******************************************************************************
  */

#define SAMPLE_SIZE 16
#define BUFFER_SIZE (SAMPLE_SIZE << 5)
#define DIFFSTEP 8

/**
  * @brief  Performs calculations to determine the heart rate and SpO2.
  *
  * @note   This function have in input arrays of raw values from red and
  *         infrared LEDs. Using them we get heart rate and SpO2.
  * @return Last calculated heart rate (beats per minute)
  */
float calculateMax30100(void);

/**
  * @brief  Reurn value of heart rate.
  *
  * @return Last calculated heart rate (beats per minute)
  */
float getHeartRate(void);

/**
  * @brief  Reurn value of coefficient R.
  *
  * @note   R = (ACred / DCred) / (ACir / DCir)
  *         where
  *         AC - pulsatile component
  *         DC - baseline component
  *         red - index of red LED (wavelength 660nm)
  *         ir - index of infrared LED (wavelength 870nm)
  *
  * @return R value
  */
float getR(void);

/**
  * @brief  Reurn value of SpO2.
  *
  * @return Last calculated SpO2 value
  */
float getSpO2(void);
