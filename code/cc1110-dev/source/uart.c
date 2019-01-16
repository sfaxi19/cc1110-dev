#include "../include/uart.h"
#include "../include/ioCCxx10_bitdef.h"
#include "../include/msg_format.h"

void uartSetup(void)
{
  // Initialize P0_1 for SRF04EB S1 button
  P0SEL &= ~BIT1;
  P0DIR &= ~BIT1;
  P0INP |= BIT1;
 

  /***************************************************************************
   * Setup I/O ports
   *
   * Port and pins used by USART0 operating in UART-mode are
   * RX     : P0_2
   * TX     : P0_3
   * CT/CTS : P0_4
   * RT/RTS : P0_5
   *
   * These pins can be set to function as peripheral I/O to be be used by UART0.
   * The TX pin on the transmitter must be connected to the RX pin on the receiver.
   * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
   * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
   * receiver.
   */

  // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
  // To avoid potential I/O conflict with USART1:
  // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
  PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

  // Configure relevant Port P0 pins for peripheral function:
  // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
  P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;



  /***************************************************************************
   * Configure UART
   *
   * The system clock source used is the HS XOSC at 26 MHz speed.
   */

  // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
  // ref. [clk]=>[clk_xosc.c]
  /*SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;*/


  // Initialise bitrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;

  // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

  // USART mode = UART (U0CSR.MODE = 1)
  U0CSR |= U0CSR_MODE;

  // Start bit level = low => Idle level = high  (U0UCR.START = 0)
  U0UCR &= ~U0UCR_START;

  // Stop bit level = high (U0UCR.STOP = 1)
  U0UCR |= U0UCR_STOP;

  // Number of stop bits = 1 (U0UCR.SPB = 0)
  U0UCR &= ~U0UCR_SPB;

  // Parity = disabled (U0UCR.PARITY = 0)
  U0UCR &= ~U0UCR_PARITY;

  // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
  U0UCR &= ~U0UCR_BIT9;

  // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
  // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
  // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
  // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
  U0UCR &= ~U0UCR_D9;

  // Flow control = disabled (U0UCR.FLOW = 0)
  U0UCR &= ~U0UCR_FLOW;

  // Bit order = LSB first (U0GCR.ORDER = 0)
  U0GCR &= ~U0GCR_ORDER;
}

void uart8Send(uint8* uartTxBuf, uint16 uartTxBufLength)
{
  uint16 uartTxIndex;
  // Clear any pending TX interrupt request (set U0CSR.TX_BYTE = 0)
  U0CSR &= ~U0CSR_TX_BYTE;
  // Loop: send each UART0 sample on the UART0 TX line
  for (uartTxIndex = 0; uartTxIndex < uartTxBufLength; uartTxIndex++)
  {
    U0DBUF = uartTxBuf[uartTxIndex];
    while(! (U0CSR&U0CSR_TX_BYTE) );
    U0CSR &= ~U0CSR_TX_BYTE;
  }
}

void uart8Receive(uint8* uartRxBuf, uint16 uartRxBufLength)
{
  uint16 uartRxIndex;
  // Enable UART0 RX (U0CSR.RE = 1)
  U0CSR |= U0CSR_RE;
  // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
  U0CSR &= ~U0CSR_RX_BYTE;
  // Loop: receive each UART0 sample from the UART0 RX line
  for (uartRxIndex = 0; uartRxIndex < uartRxBufLength; uartRxIndex++)
  {
    // Wait until data received (U0CSR.RX_BYTE = 1)
    while( !(U0CSR&U0CSR_RX_BYTE) );
    // Read UART0 RX buffer
    uartRxBuf[uartRxIndex] = U0DBUF;
  }
}

void uart16Receive(uint16* uartRxBuf, uint16 uartRxBufLength)
{
  uint16 uartRxIndex;
  // Enable UART0 RX (U0CSR.RE = 1)
  U0CSR |= U0CSR_RE;
  // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
  U0CSR &= ~U0CSR_RX_BYTE;
  // Loop: receive each UART0 sample from the UART0 RX line
  for (uartRxIndex = 0; uartRxIndex < uartRxBufLength; uartRxIndex++)
  {
    // Wait until data received (U0CSR.RX_BYTE = 1)
    while( !(U0CSR&U0CSR_RX_BYTE) );
    // Read UART0 RX buffer
    uartRxBuf[uartRxIndex] = U0DBUF;
  }
}
