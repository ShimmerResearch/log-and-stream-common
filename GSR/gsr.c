/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date December, 2013
 *
 * @edited Sam O'Mahony
 * @date December, 2017
 *
 */

#include "gsr.h"

#include "math.h"

#if defined(SHIMMER3)
#include "msp430.h"
#else
#include "hal_Board.h"
#include "stm32u5xx_hal.h"
#endif

#define HW_RES_40K_MIN_ADC_VAL \
  1120 //10k to 56k..1159->1140 //nom: changed to 1120 for linear conversion

#define HW_RES_287K_MAX_ADC_VAL \
  3960 //56k to 220k was 4000 but was 3948 on shimmer so changed to 3800 //nom: changed to 3960 for linear conversion
#define HW_RES_287K_MIN_ADC_VAL 1490 //56k to 220k..1510->1490

#define HW_RES_1M_MAX_ADC_VAL   3700 //220k to 680k
#define HW_RES_1M_MIN_ADC_VAL   1630 //220k to 680k..1650->1630

#define HW_RES_3M3_MAX_ADC_VAL  3930 //680k to 4M7
#define HW_RES_3M3_MIN_ADC_VAL  1125 //680k to 4M7

//when we switch resistors with the ADG658 it takes a few samples for the
//ADC to start to see the new sampled voltage correctly, the catch below is
//to eliminate any glitches in the data
#define ONE_HUNDRED_OHM_STEP    100
#define MAX_RESISTANCE_STEP     5000
//instead of having a large step when resistors change - have a smoother step
#define NUM_SMOOTHING_SAMPLES   64
//ignore these samples after a resistor switch - instead send special code
#define NUM_SAMPLES_TO_IGNORE   6
#define STARTING_RESISTANCE     10000000
//Settling time for a hardware resistor change (80 ms)
#define SETTLING_TIME           2621 //32768*0.08=2621.44

uint8_t last_active_resistor, transient_active_resistor, got_first_sample;
uint16_t transient_sample, transient_smoothing_samples, max_resistance_step;
uint32_t last_resistance;
uint8_t gsrRangePinsAreReversed = 0;

uint8_t gsrRangeConfigured;
uint8_t gsrActiveResistor;
uint16_t lastGsrVal;
uint16_t gsrSamplingRateTicks;

void GSR_init(uint8_t gsrRangeToSet, uint16_t gsrSamplingRateTicksToSet)
{
  gsrRangeConfigured = gsrRangeToSet;
  if (gsrRangeConfigured <= HW_RES_3M3)
  {
    GSR_setActiveResistor(gsrRangeConfigured);
  }
  else
  {
    GSR_setActiveResistor(HW_RES_40K);
  }

  gsrSamplingRateTicks = gsrSamplingRateTicksToSet;

  lastGsrVal = 0;
  GSR_initSmoothing(gsrActiveResistor);
}

void GSR_range(uint8_t *buf)
{
  //Fill the current active resistor into the upper two bits of the GSR value
  //if autorange is enabled, switch active resistor if required
  //If during resistor transition period use old ADC and resistor values
  //as determined by GSR_smoothTransition()

  uint8_t current_active_resistor = gsrActiveResistor;
  uint16_t ADC_val;

  ADC_val = *((uint16_t *) buf);
  if (gsrRangeConfigured == GSR_AUTORANGE)
  {
    if (GSR_smoothTransition(&current_active_resistor, gsrSamplingRateTicks))
    {
      ADC_val = lastGsrVal;
    }
    else
    {
      GSR_controlRange(ADC_val);
    }
  }

  //filling the upper two bits with the current value of selected GSR resistor
  *((uint16_t *) buf) = ADC_val | (current_active_resistor << 14);

  lastGsrVal = ADC_val;
}

void GSR_setActiveResistor(uint8_t range)
{
  switch (range)
  {
    case HW_RES_40K:
      GSR_setA0(0);
      GSR_setA1(0);
      break;

    case HW_RES_287K:
      if (gsrRangePinsAreReversed)
      {
        GSR_setA0(0);
        GSR_setA1(1);
      }
      else
      {
        GSR_setA0(1);
        GSR_setA1(0);
      }
      break;

    case HW_RES_1M:
      if (gsrRangePinsAreReversed)
      {
        GSR_setA0(1);
        GSR_setA1(0);
      }
      else
      {
        GSR_setA0(0);
        GSR_setA1(1);
      }
      break;

    case HW_RES_3M3:
      GSR_setA0(1);
      GSR_setA1(1);
      break;
  }
  gsrActiveResistor = range;
}

void GSR_setA0(uint8_t state)
{
#if defined(SHIMMER3)
  if (state)
  {
    P1OUT |= BIT4;
  }
  else
  {
    P1OUT &= ~BIT4;
  }
#else
  HAL_GPIO_WritePin(GSR_RANGE_A0_GPIO_Port, GSR_RANGE_A0_Pin,
      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

void GSR_setA1(uint8_t state)
{
#if defined(SHIMMER3)
  if (state)
  {
    P2OUT |= BIT1;
  }
  else
  {
    P2OUT &= ~BIT1;
  }
#else
  HAL_GPIO_WritePin(GSR_RANGE_A1_GPIO_Port, GSR_RANGE_A1_Pin,
      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint64_t multiply(uint64_t no1, uint64_t no2)
{
  if (no1 == 0 || no2 == 0)
  {
    return 0;
  }
  if (no1 == 1)
  {
    return no2;
  }
  if (no2 == 1)
  {
    return no1;
  }
  return no1 * no2;
}

uint32_t GSR_calcResistance(float gsrMv)
{
  //Uses op amp equation
  return GSR_FEEDBACK_RESISTORS_OHMS[gsrActiveResistor] / (((gsrMv / 1000) / 0.5) - 1.0);
}

void GSR_controlRange(uint16_t ADC_val)
{
  switch (gsrActiveResistor)
  {
    case HW_RES_40K:
      if (ADC_val < HW_RES_40K_MIN_ADC_VAL)
      {
        GSR_setActiveResistor(HW_RES_287K);
      }
      break;
    case HW_RES_287K:
      if ((ADC_val <= HW_RES_287K_MAX_ADC_VAL) && (ADC_val >= HW_RES_287K_MIN_ADC_VAL))
      {
        ; //stay here
      }
      else if (ADC_val < HW_RES_287K_MIN_ADC_VAL)
      {
        GSR_setActiveResistor(HW_RES_1M);
      }
      else
      {
        GSR_setActiveResistor(HW_RES_40K);
      }
      break;
    case HW_RES_1M:
      if ((ADC_val <= HW_RES_1M_MAX_ADC_VAL) && (ADC_val >= HW_RES_1M_MIN_ADC_VAL))
      {
        ; //stay here
      }
      else if (ADC_val < HW_RES_1M_MIN_ADC_VAL)
      {
        GSR_setActiveResistor(HW_RES_3M3);
      }
      else
      {
        GSR_setActiveResistor(HW_RES_287K);
      }
      break;
    case HW_RES_3M3:
      if ((ADC_val <= HW_RES_3M3_MAX_ADC_VAL) && (ADC_val >= HW_RES_3M3_MIN_ADC_VAL))
      {
        ; //stay here
      }
      else if (ADC_val > HW_RES_3M3_MAX_ADC_VAL)
      {
        GSR_setActiveResistor(HW_RES_1M);
      }
      else
      {
        //MIN so can't go any higher
      }
      break;
    default:;
  }
}

void GSR_initSmoothing(uint8_t active_resistor)
{
  last_active_resistor = active_resistor;
  got_first_sample = 0;
  transient_sample = NUM_SAMPLES_TO_IGNORE;
  transient_smoothing_samples = 0;
  max_resistance_step = MAX_RESISTANCE_STEP;
  last_resistance = STARTING_RESISTANCE;
}

//Smooth the transition without converting ADC value to resistance:
//Repeat the last 'valid' ADC value for each sample during the settling period,
//then step to the next valid ADC value after settling. This will not be seen
//by most GSR applications as the settling time (approx 40 ms) is smaller than
//typical GSR 'event' frequencies. If sampling at a high rate (e.g. due to other
//active sensors, this function will prevent large spikes in the GSR due to settling.
uint8_t GSR_smoothTransition(uint8_t *dummy_active_resistor, uint32_t sampling_period)
{
  //Number of 'transient' samples proportional to sampling rate.
  if (*dummy_active_resistor != last_active_resistor)
  {
    if (transient_sample && (transient_active_resistor == *dummy_active_resistor))
    {
      transient_sample = 0;
    }
    else
    {
      transient_sample = ceil(SETTLING_TIME / sampling_period);
      transient_active_resistor = last_active_resistor;
    }
  }
  last_active_resistor = *dummy_active_resistor;
  if (transient_sample)
  {
    transient_sample--;
    //keep the previous active resistor in the buffer during transition settling time
    *dummy_active_resistor = transient_active_resistor;
    return 1;
  }
  return 0;
}

uint32_t GSR_smoothSample(uint32_t resistance, uint8_t active_resistor)
{
  if (active_resistor != last_active_resistor)
  {
    transient_sample = NUM_SAMPLES_TO_IGNORE;
    max_resistance_step = ONE_HUNDRED_OHM_STEP;
    transient_smoothing_samples = NUM_SMOOTHING_SAMPLES;
    last_active_resistor = active_resistor;
  }
  //if we are after a transition then max_resistance_step will be small to ensure smooth transition
  if (transient_smoothing_samples)
  {
    transient_smoothing_samples--;
    //if we are finished smoothing then go back to a larger resistance step
    if (!transient_smoothing_samples)
    {
      max_resistance_step = MAX_RESISTANCE_STEP;
    }
  }
  //only prevent a large step from last resistance if we actually have a last resistance
  if ((got_first_sample) && (last_resistance > max_resistance_step))
  {
    if (resistance > (last_resistance + max_resistance_step))
    {
      resistance = (last_resistance + max_resistance_step);
    }
    else if (resistance < (last_resistance - max_resistance_step))
    {
      resistance = (last_resistance - max_resistance_step);
    }
    else
      ;
  }
  else
  {
    //get the first sample in this run of sampling
    got_first_sample = 1;
  }

  last_resistance = resistance;

  //if this sample is near a resistor transition then send a special code for data analysis
  if (transient_sample)
  {
    transient_sample--;
    resistance = 0xFFFFFFFF;
  }

  return resistance;
}

void setGsrRangePinsAreReversed(uint8_t state)
{
  gsrRangePinsAreReversed = state;
}
