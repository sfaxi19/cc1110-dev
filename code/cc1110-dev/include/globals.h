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

#define DMAARM_CHANNEL1              0x02   // The value to arm the DMA
                                            // channel 1 in the DMAARM register
#define DMA_RADIO_TX_CHANNEL         DMAARM_CHANNEL0
#define DMA_RADIO_RX_CHANNEL         DMAARM_CHANNEL1

#define RADIO_MODE_TX                0x10
#define RADIO_MODE_RX                0x20


extern BYTE PACKET_LENGTH;
extern BYTE radioPktBuffer[MAX_PACKET_LENGTH];

extern DMA_DESC dmaTxConfig0;
extern DMA_DESC dmaRxConfig1;

extern BYTE mode;                           // Radio operating mode, either RX or TX


extern void dmaRadioSetup(UINT8 mode);
extern void radioSettingsApply(settings_s* settings);
extern void radioConfigure(settings_s* settings);
extern INT16 convertRssiByte(BYTE RSSI_value);
extern void showLogo(void);

#endif //GLOBALS_H
