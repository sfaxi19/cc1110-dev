/*-----------------------------------------------------------------------------
|   File:      hal_adc_mgmt.h
|   Target:    cc1110, cc2510
|   Author:    TFL
|   Revised:   2007-09-05
|   Revision:  1.0
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
| Purpose:    ADC management
+------------------------------------------------------------------------------
| Decription:
|     These functions/macros simplifies usage of the ADC.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_ADC_MGMT_H
#define HAL_ADC_MGMT_H

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== CONSTS ================================================================*/

// Values used for the _settings_ in macros / functions below

// Reference voltage:
#define ADC_REF_1_25_V      0x00     // Internal 1.25V reference
#define ADC_REF_P0_7        0x40     // External reference on AIN7 pin
#define ADC_REF_AVDD        0x80     // AVDD_SOC pin
#define ADC_REF_P0_6_P0_7   0xC0     // External reference on AIN6-AIN7 differential input

// Resolution (decimation rate):
#define ADC_7_BIT           0x00     //  64 decimation rate
#define ADC_9_BIT           0x10     // 128 decimation rate
#define ADC_10_BIT          0x20     // 256 decimation rate
#define ADC_12_BIT          0x30     // 512 decimation rate

// Input channel:
#define ADC_AIN0            0x00     // single ended P0_0
#define ADC_AIN1            0x01     // single ended P0_1
#define ADC_AIN2            0x02     // single ended P0_2
#define ADC_AIN3            0x03     // single ended P0_3
#define ADC_AIN4            0x04     // single ended P0_4
#define ADC_AIN5            0x05     // single ended P0_5
#define ADC_AIN6            0x06     // single ended P0_6
#define ADC_AIN7            0x07     // single ended P0_7
#define ADC_GND             0x0C     // Ground
#define ADC_TEMP_SENS       0x0E     // on-chip temperature sensor
#define ADC_VDD_3           0x0F     // (vdd/3)

// Bit masks used for ADCCON1 ADC control 1
#define ADCCON1_ST_START    0x70     // Starting conversion
#define ADCCON1_ST_NORMAL   0x03     // Normal Operation

/*==== TYPES =================================================================*/

/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

// Macro for setting up a single conversion. If ADCCON1.STSEL = 11, using this
// macro will also start the conversion.
#define ADC_SINGLE_CONVERSION(settings) \
  do{                                   \
    ADCCON3 = (settings);               \
  }while(0)

// Macro for setting up a single conversion
#define ADC_SEQUENCE_SETUP(settings)    \
  do{                                   \
    ADCCON2 = (settings);               \
  }while(0)

// Macro for starting the ADC in continuous conversion mode
#define ADC_SAMPLE_CONTINUOUS()   \
  do {                            \
    ADCCON1 &= ~0x30;             \
    ADCCON1 |= 0x10;              \
  } while (0)

// Macro for stopping the ADC in continuous mode
#define ADC_STOP()                \
  do {                            \
    ADCCON1 |= 0x30;              \
  } while (0)

// Macro for initiating a single sample in single-conversion mode (ADCCON1.STSEL = 11).
#define ADC_SAMPLE_SINGLE()       \
  do{                             \
    ADC_STOP();                   \
    ADCCON1 |= 0x40;              \
} while (0)

// Macro for configuring the ADC to be started from T1 channel 0. (T1 ch 0 must be in compare mode!!)
#define ADC_TRIGGER_FROM_TIMER1() \
  do {                            \
    ADC_STOP();                   \
    ADCCON1 &= ~0x10;             \
  } while (0)

// Expression indicating whether a conversion is finished or not.
#define ADC_SAMPLE_READY()      (ADCCON1 & 0x80)

// Macro for setting/clearing a channel as input of the ADC
#define ADC_ENABLE_CHANNEL(ch)   ADCCFG |=  (0x01 << ch)
#define ADC_DISABLE_CHANNEL(ch)  ADCCFG &= ~(0x01 << ch)

// Macro for getting the ADC results
#define ADC_GET_VALUE( v )       GET_WORD( ADCH, ADCL, v )

/*==== FUNCTIONS =============================================================*/

/******************************************************************************
* @fn  halAdcSampleSingle
*
* @brief
*      This function makes the adc sample the given channel at the given
*      resolution with the given reference.
*
* Parameters:
*
* @param BYTE reference
*          The reference to compare the channel to be sampled.
*        BYTE resolution
*          The resolution to use during the sample (7, 9, 10 or 12 bit)
*        BYTE input
*          The channel to be sampled.
*
* @return INT16
*          The conversion result (rightbound; see hal_adc_mgmt.c for details)
*
******************************************************************************/
INT16 halAdcSampleSingle(BYTE reference, BYTE resolution, UINT8 input);

#endif /* HAL_ADC_MGMT_H */

/*==== END OF FILE ==========================================================*/
