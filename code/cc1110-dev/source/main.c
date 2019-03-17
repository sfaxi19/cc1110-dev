#include "../../Library/HAL/include/hal_main.h"
#include "../include/ioCCxx10_bitdef.h"

#include "../include/globals.h"
#include "../include/uart.h"
#include "../include/uart_fsm.h"
#include "../include/msg_format.h"

BYTE PACKET_LENGTH;

#define PACKET_RECEIVE_TIMEOUT -1
#define PACKET_RECEIVE_UART 0
#define PACKET_RECEIVE_RADIO 1
#define PACKET_ERROR_UART 2

static char log[2][16];

static BOOL radioSentFlag = FALSE;            // Flag set whenever a packet is sent
static BOOL radioRecvFlag = FALSE;            // Flag set whenever a packet is received

BOOL IsUartRecv()
{	// Data received (U0CSR.RX_BYTE = 1)
	return U0CSR & U0CSR_RX_BYTE;
}

BOOL IsRadioRecv()
{
	return radioRecvFlag;
}

void blik1()
{
	static BOOL led_status = TRUE;
	if (led_status) LED1 = LED_ON;
	else LED1 = LED_OFF;
	led_status = !led_status;
}

void blik3()
{
	static BOOL led_status = TRUE;
	if (led_status) LED3 = LED_ON;
	else LED3 = LED_OFF;
	led_status = !led_status;
}

int packetReceiving(uint32 timeout)
{
	blik1();
	
	if (mode == RADIO_MODE_RX)
	{
		//Enable radio receive
		radioRecvFlag = FALSE;
		DMAARM |= DMA_RADIO_RX_CHANNEL;
		RFST = STROBE_RX;
	}
	//Enable uart receive
	U0CSR |= U0CSR_RE;       // Enable UART0 RX (U0CSR.RE = 1)
	U0CSR &= ~U0CSR_RX_BYTE; // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
		
	if (halBuiButtonPushed())
	{
		char log[2][16];
		sprintf(&log[0][0], "TIMEOUT: %u", timeout);
		sprintf(&log[1][0], "RADIO_STATE: %u", MARCSTATE);
		halBuiLcdUpdate(log[0], log[1]);
	}
	
	if (MARCSTATE == MARC_STATE_RX_OVERFLOW)
	{
		RFST = RFST_SIDLE;
		RFST = RFST_SRX;
		halBuiLcdUpdate("      RX       ", 
						"BAD RADIO STATE");
	}
	
	uint32 timer = 0;
	
	while( !IsUartRecv() && !IsRadioRecv() ) 
	{
		if (timer++ > timeout) return PACKET_RECEIVE_TIMEOUT;
	}
	timer = 0;
	
	if (IsUartRecv())
	{
		uint16 rxIndex;

		// read header
		for (rxIndex = 0; rxIndex < sizeof(proto_s); rxIndex++)
		{
			while( !IsUartRecv() ) 
			{
				if (timer++ > timeout) return PACKET_RECEIVE_TIMEOUT;
			}
			// Read UART0 RX buffer
			uartPktBuffer[rxIndex] = U0DBUF;
			U0CSR &= ~U0CSR_RX_BYTE; // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
		}
		proto_s* proto = (proto_s*)uartPktBuffer;
		
		uint16 size = sizeof(proto_s) + proto->data_size + sizeof(crc_t);
		
		if (size > UART_BUFFER_SIZE)
		{
			return PACKET_ERROR_UART;
		}

		for (; rxIndex < size; rxIndex++)
		{
			while( !IsUartRecv() )
			{
				if (++timer > timeout) return PACKET_RECEIVE_TIMEOUT;
			}
			// Read UART0 RX buffer
			uartPktBuffer[rxIndex] = U0DBUF;
			U0CSR &= ~U0CSR_RX_BYTE; // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
		}
		
		BOOL crc_valid = decode(uartPktBuffer, sizeof(proto_s) + proto->data_size);
		if (!crc_valid)
		{
			return PACKET_ERROR_UART;
		}
			
		return PACKET_RECEIVE_UART;
	}
	else if (IsRadioRecv())
	{
		return PACKET_RECEIVE_RADIO;
	}
}

void radioSending(uint32 transmissions)
{
	uint32 pktCnt = 0;
	while((pktCnt++ < transmissions) && !IsUartRecv())
	{
		if (MARCSTATE == MARC_STATE_TX_UNDERFLOW)
		{
			RFST = RFST_SIDLE;
			RFST = RFST_STX;
			halBuiLcdUpdate("      TX        ", 
							"BAD RADIO STATE");
		}
		// Send the packet
		DMAARM |= DMA_RADIO_TX_CHANNEL;  // Arm DMA channel 0
		RFST = STROBE_TX;           // Switch radio to TX
		
		// Wait until the radio transfer is completed,
		// and then reset radioSentFlag
		while(!radioSentFlag)
		{
			if (halBuiButtonPushed())
			{
				char log[2][16];
				sprintf(&log[0][0], "%d / %u", pktCnt, transmissions);
				sprintf(&log[1][0], "RADIO_STATE: %u", MARCSTATE);
				halBuiLcdUpdate(log[0], log[1]);
			}
		}
		radioSentFlag = FALSE;
		if (transmissions > 1)
		{
			halWait(30);
		}
	}
}

void radioSettingsApply(settings_s* settings)
{
	radioConfigure(settings);

	DMAARM = 0x80;
	INT_GLOBAL_ENABLE(INT_OFF);          // Disable interrupts globally
	DMAARM &=0x80;
	RFST = RFST_SIDLE;
	HAL_INT_ENABLE(INUM_RF, INT_OFF);    // Disable RF general interrupt
	RFIM = 0;
	
	switch (mode)
	{
	case RADIO_MODE_TX:
		// Set up the DMA to move packet data from buffer to radio
		dmaRadioSetup(RADIO_MODE_TX);
		// Configure interrupt for every time a packet is sent
		HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
		RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
		INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
		break; 
	case RADIO_MODE_RX:
		dmaRadioSetup(RADIO_MODE_RX);
		// Configure interrupt for every received packet
		HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
		RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
		INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
		
		// Start receiving
		DMAARM |= DMA_RADIO_RX_CHANNEL;           // Arm DMA channel 0
		RFST   = STROBE_RX;                 // Switch radio to RX
		break;
	default:
		break;
	}
}

void main(void)
{
	INIT_LED1();
	INIT_LED3();
	INIT_BUTTON();
	
	// Choose the crystal oscillator as the system clock
	halPowerClkMgmtSetMainClkSrc(CRYSTAL);
	
	halBuiInitLcd();
	
	showLogo();
	
	//while(!halBuiButtonPushed());
	
	uartSetup();
	eState m_state = idle_state;
	m_state = StateMachineTable[m_state].StartHandler(m_state);
	
	while(TRUE)
	{
		int res = packetReceiving(10000);
		
		switch (res)
		{
		case PACKET_RECEIVE_UART:
		{
			//proto_s* msg_header = (proto_s*) rx_buffer;
			//sprintf(&log[0][0], "CMD  : %s", TypeToString(msg_header->msg_type));
			//sprintf(&log[1][0], "STATE: %s", toString(m_state));
			//halBuiLcdUpdate(log[0], log[1]);
			m_state = StateMachineTable[m_state].ReceiveHandler(m_state, uartPktBuffer);
			break;
		}
		case PACKET_RECEIVE_RADIO:
		{
			static int radio_recv_cnt = 0;
			m_state = StateMachineTable[m_state].RadioReceiveHandler(m_state, radioPktRxBuffer1);
			//sprintf(&log[0][0], "   Radio[%d] ", ++radio_recv_cnt);
			//sprintf(&log[1][0], "Packet receive");
			//halBuiLcdUpdate(log[0], log[1]);
			break;
		}
		case PACKET_RECEIVE_TIMEOUT:
		{
			m_state = StateMachineTable[m_state].TimeoutHandler(m_state);
			break;
		}
		case PACKET_ERROR_UART:
		{
			halBuiLcdUpdate("      CRC       ", 
							"     ERROR!     ");
			break;
		}
		default:
			break;
		}
	}
}

#pragma vector=RF_VECTOR
__interrupt void rf_IRQ(void) {
	RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
	S1CON &= ~0x03;           // Clear the general RFIF interrupt registers
	
	blik3();
	
	if (mode == RADIO_MODE_RX) {
		radioRecvFlag = TRUE;
	}
	else {
		radioSentFlag = TRUE;
	}
}

/*==== END OF FILE ==========================================================*/
