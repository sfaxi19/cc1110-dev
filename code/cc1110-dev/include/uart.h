#include "../../Library/HAL/include/hal_main.h"
#include "../include/dma.h"

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
#define UART_BAUD_E  11

void uartSetup();
void uart16Send(uint16* uartTxBuf, uint16 uartTxBufLength);
void uart16Receive(uint16* uartRxBuf, uint16 uartRxBufLength);

void uart8Send(uint8* uartTxBuf, uint16 uartTxBufLength);
void uart8Receive(uint8* uartRxBuf, uint16 uartRxBufLength);