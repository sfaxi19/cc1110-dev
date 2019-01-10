/*-----------------------------------------------------------------------------
|   File:      per_test_radio.c
|   Target:    cc1110, cc2510
|   Author:    ESY
|   Revised:   2007-09-06
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
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
| Purpose:    Functions for radio and packet handling for PER test
+------------------------------------------------------------------------------
| Decription: All functions related to radio configuration and packet
|             handling for the packet error rate test application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "../../Library/HAL/include/hal_main.h"
#include "../include/globals.h"
#include "../include/msg_format.h"


/*==== GLOBAL VARS ===========================================================*/

/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* @fn  radioConfigure
******************************************************************************/
void radioConfigure(settings_s* settings)
{
    mode        = settings->MODE;
    SYNC1       = settings->CC1110_SYNC1;        /*  Sync word, high byte                                */
    SYNC0       = settings->CC1110_SYNC0;        /*  Sync word, low byte                                 */
    PKTLEN      = settings->CC1110_PKTLEN;       /*  Packet length                                       */
    PKTCTRL1    = settings->CC1110_PKTCTRL1;     /*  Packet automation control                           */
    PKTCTRL0    = settings->CC1110_PKTCTRL0;     /*  Packet automation control                           */
    ADDR        = settings->CC1110_ADDR;         /*  Device address                                      */
    CHANNR      = settings->CC1110_CHANNR;       /*  Channel number                                      */
    FSCTRL1     = settings->CC1110_FSCTRL1;      /*  Frequency synthesizer control                       */
    FSCTRL0     = settings->CC1110_FSCTRL0;      /*  Frequency synthesizer control                       */
    FREQ2       = settings->CC1110_FREQ2;        /*  Frequency control word, high byte                   */
    FREQ1       = settings->CC1110_FREQ1;        /*  Frequency control word, middle byte                 */
    FREQ0       = settings->CC1110_FREQ0;        /*  Frequency control word, low byte                    */
    MDMCFG4     = settings->CC1110_MDMCFG4;      /*  Modem configuration                                 */
    MDMCFG3     = settings->CC1110_MDMCFG3;      /*  Modem configuration                                 */
    MDMCFG2     = settings->CC1110_MDMCFG2;      /*  Modem configuration                                 */
    MDMCFG1     = settings->CC1110_MDMCFG1;      /*  Modem configuration                                 */
    MDMCFG0     = settings->CC1110_MDMCFG0;      /*  Modem configuration                                 */
    DEVIATN     = settings->CC1110_DEVIATN;      /*  Modem deviation setting                             */
    MCSM2       = settings->CC1110_MCSM2;        /*  Main Radio Control State Machine configuration      */
    MCSM1       = settings->CC1110_MCSM1;        /*  Main Radio Control State Machine configuration      */
    MCSM0       = settings->CC1110_MCSM0;        /*  Main Radio Control State Machine configuration      */
    FOCCFG      = settings->CC1110_FOCCFG;       /*  Frequency Offset Compensation configuration         */
    BSCFG       = settings->CC1110_BSCFG;        /*  Bit Synchronization configuration                   */
    AGCCTRL2    = settings->CC1110_AGCCTRL2;     /*  AGC control                                         */
    AGCCTRL1    = settings->CC1110_AGCCTRL1;     /*  AGC control                                         */
    AGCCTRL0    = settings->CC1110_AGCCTRL0;     /*  AGC control                                         */
    FREND1      = settings->CC1110_FREND1;       /*  Front end RX configuration                          */
    FREND0      = settings->CC1110_FREND0;       /*  Front end TX configuration                          */
    FSCAL3      = settings->CC1110_FSCAL3;      /*  Frequency synthesizer calibration                   */
    FSCAL2      = settings->CC1110_FSCAL2;      /*  Frequency synthesizer calibration                   */
    FSCAL1      = settings->CC1110_FSCAL1;      /*  Frequency synthesizer calibration                   */
    FSCAL0      = settings->CC1110_FSCAL0;      /*  Frequency synthesizer calibration                   */
    
    //FSTEST      = settings->CC1110_FSTEST;      /*  Frequency synthesizer calibration control           */
    //PTEST       = settings->CC1110_PTEST;       /*  Production test                                     */
    //AGCTEST     = settings->CC1110_AGCTEST;     /*  AGC test                                            */
    
    TEST2       = settings->CC1110_TEST2;       /*  Various test settings                               */
    TEST1       = settings->CC1110_TEST1;       /*  Various test settings                               */
    TEST0       = settings->CC1110_TEST0;       /*  Various test settings                               */
    PA_TABLE7   = settings->CC1110_PA_TABLE7;   /*  PA power setting 7                                  */
    PA_TABLE6   = settings->CC1110_PA_TABLE6;   /*  PA power setting 6                                  */
    PA_TABLE5   = settings->CC1110_PA_TABLE5;   /*  PA power setting 5                                  */
    PA_TABLE4   = settings->CC1110_PA_TABLE4;   /*  PA power setting 4                                  */
    PA_TABLE3   = settings->CC1110_PA_TABLE3;   /*  PA power setting 3                                  */
    PA_TABLE2   = settings->CC1110_PA_TABLE2;   /*  PA power setting 2                                  */
    PA_TABLE1   = settings->CC1110_PA_TABLE1;   /*  PA power setting 1                                  */
    PA_TABLE0   = settings->CC1110_PA_TABLE0;   /*  PA power setting 0                                  */
    IOCFG2      = settings->CC1110_IOCFG2;      /*  Radio Test Signal Configuration (P1_7)              */
    IOCFG1      = settings->CC1110_IOCFG1;      /*  Radio Test Signal Configuration (P1_6)              */
    IOCFG0      = settings->CC1110_IOCFG0;      /*  Radio Test Signal Configuration (P1_5)              */
    PARTNUM     = settings->CC1110_PARTNUM;     /*  Chip ID [15:8]                                      */
    VERSION     = settings->CC1110_VERSION;     /*  Chip ID [7:0]                                       */
    FREQEST     = settings->CC1110_FREQEST;     /*  Frequency Offset Estimate                           */
    LQI         = settings->CC1110_LQI;         /*  Link Quality Indicator                              */
    RSSI        = settings->CC1110_RSSI;        /*  Received Signal Strength Indication                 */
    MARCSTATE   = settings->CC1110_MARCSTATE;   /*  Main Radio Control State                            */
    PKTSTATUS   = settings->CC1110_PKTSTATUS;   /*  Packet status                                       */
    VCO_VC_DAC  = settings->CC1110_VCO_VC_DAC;  /*  PLL calibration current                             */
}

/*==== END OF FILE ==========================================================*/
