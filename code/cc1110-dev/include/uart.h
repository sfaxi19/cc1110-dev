#include "../../Library/HAL/include/hal_main.h"
#include "../include/dma.h"

/***********************************************************************************
* CONSTANTS
*/


// Define size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

#define UART_TST_CHAR_1  0xA5
#define UART_TST_CHAR_2  0xB5

// Test definitions
//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
#define UART_BAUD_E  11

#define PACKET_RECEIVE_TIMEOUT -1
#define PACKET_RECEIVE_UART 0
#define PACKET_RECEIVE_RADIO 1



/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer for UART RX/TX
static uint16 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint16 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];

// Variable for buffer indexing
static uint16 __xdata i;

void uartSetup();
void uart16Send(uint16* uartTxBuf, uint16 uartTxBufLength);
void uart16Receive(uint16* uartRxBuf, uint16 uartRxBufLength);

void uart8Send(uint8* uartTxBuf, uint16 uartTxBufLength);
void uart8Receive(uint8* uartRxBuf, uint16 uartRxBufLength);

int packetReceiver(BOOL* radioRecvFlag, uint8* buffer, uint16 buffer_size, uint32 timeout);