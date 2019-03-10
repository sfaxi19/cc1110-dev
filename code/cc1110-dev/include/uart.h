#include "../../Library/HAL/include/hal_main.h"
#include "../include/dma.h"

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
#define UART_BAUD_E  11

typedef uint16 crc_t;

crc_t crc16(uint8* data, uint16 size);
BOOL decode(uint8* data);

void uartSetup();
void uart8Send(uint8* uartTxBuf, uint16 uartTxBufLength);
void uart8Receive(uint8* uartRxBuf, uint16 uartRxBufLength);