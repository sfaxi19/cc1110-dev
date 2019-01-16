#ifndef GLOBALS_H
#define GLOBALS_H

#include "dma.h"
#include "msg_format.h"

#define MAX_PACKET_LENGTH            0xFF   

#define STROBE_TX                    0x03   // Strobe commands for the RFST
#define STROBE_RX                    0x02   // register

#define IRQ_DONE                     0x10   // The IRQ_DONE bit in the RFIF-
                                            // and RFIM-register
#define DMAARM_CHANNEL0              0x01   // The value to arm the DMA
                                            // channel 0 in the DMAARM register
#define RADIO_MODE_TX                0x10
#define RADIO_MODE_RX                0x20


extern BYTE PACKET_LENGTH;
extern BYTE radioPktBuffer[MAX_PACKET_LENGTH];
static BOOL pktSentFlag = FALSE;            // Flag set whenever a packet is sent
static BOOL pktRcvdFlag = FALSE;            // Flag set whenever a packet is received
static BYTE mode;                           // Radio operating mode, either RX or TX
static DMA_DESC dmaConfig;                  // Struct for the DMA configuration

extern void dmaRadioSetup(UINT8 mode);
extern void radioSettingsApply(settings_s* settings);
extern void radioConfigure(settings_s* settings);
extern INT16 convertRssiByte(BYTE RSSI_value);
extern void showLogo(void);

#endif //GLOBALS_H
