#include "../../Library/HAL/include/hal_main.h"
#include "../include/ioCCxx10_bitdef.h"

#include "../include/globals.h"
#include "../include/uart.h"
#include "../include/uart_fsm.h"
#include "../include/msg_format.h"

#define RX_BUFFER_SIZE 1024
uint8 rx_buffer[RX_BUFFER_SIZE];

BYTE PACKET_LENGTH;
uint32 TRANSMISSIONS = 1;

#define PACKET_RECEIVE_TIMEOUT -1
#define PACKET_RECEIVE_UART 0
#define PACKET_RECEIVE_RADIO 1


int packetReceiving(BOOL* radioRecvFlag, uint8* buffer, uint16 buffer_size, uint32 timeout)
{
	static BOOL led_status = TRUE;
	if (led_status) LED1 = LED_ON;
	else LED1 = LED_OFF;
	led_status = !led_status;
	
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
	}
	uint16 rxIndex;
	// Enable UART0 RX (U0CSR.RE = 1)
	U0CSR |= U0CSR_RE;
	// Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
	U0CSR &= ~U0CSR_RX_BYTE;
	uint32 timer = 0;
	// read header
	for (rxIndex = 0; rxIndex < sizeof(proto_s); rxIndex++)
	{
		// Wait until data received (U0CSR.RX_BYTE = 1)
		while( !(U0CSR&U0CSR_RX_BYTE) ) 
		{
			if (timer++ > timeout) return PACKET_RECEIVE_TIMEOUT;
			if (*radioRecvFlag) return PACKET_RECEIVE_RADIO;
		}
		// Read UART0 RX buffer
		buffer[rxIndex] = U0DBUF;
	}
	uint16 size = rxIndex + ((proto_s*)buffer)->data_size;
	if (size > buffer_size)
	{
		size = buffer_size;
	}
	// Loop: receive each UART0 sample from the UART0 RX line
	for (; rxIndex < size; rxIndex++)
	{
		// Wait until data received (U0CSR.RX_BYTE = 1)
		while( !(U0CSR&U0CSR_RX_BYTE) )
		{
			if (timer++ > timeout) return PACKET_RECEIVE_TIMEOUT;
		}
		// Read UART0 RX buffer
		buffer[rxIndex] = U0DBUF;
	}
	return PACKET_RECEIVE_UART;
}

void radioSending()
{
	uint32 pktCnt = 0;
	while(pktCnt++ < TRANSMISSIONS)
	{
		// Send the packet
		DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
		RFST = STROBE_TX;           // Switch radio to TX
		
		// Wait until the radio transfer is completed,
		// and then reset pktSentFlag
		while(!pktSentFlag);
		pktSentFlag = FALSE;
		if (TRANSMISSIONS > 1)
		{
			halWait(50);
		}
	}
}

void radioSettingsApply(settings_s* settings)
{
	PACKET_LENGTH = settings->CC1110_PKTLEN;
	TRANSMISSIONS = settings->TRANSMISSIONS;
	radioConfigure(settings);
	switch (settings->MODE)
	{
	case RADIO_MODE_TX:
		mode = RADIO_MODE_TX;
		// Set up the DMA to move packet data from buffer to radio
		dmaRadioSetup(RADIO_MODE_TX);
		// Configure interrupt for every time a packet is sent
		HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
		RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
		INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
		break;
	case RADIO_MODE_RX:
		mode = RADIO_MODE_RX;
		dmaRadioSetup(RADIO_MODE_RX);
		// Configure interrupt for every received packet
		HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
		RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
		INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
		
		// Start receiving
		DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
		RFST   = STROBE_RX;                 // Switch radio to RX
		break;
	default:
		break;
	}
}


void main(void)
{
	char log[2][16];
	INIT_LED1();
	INIT_LED3();
	INIT_BUTTON();
	
	// Choose the crystal oscillator as the system clock
	halPowerClkMgmtSetMainClkSrc(CRYSTAL);
	
	halBuiInitLcd();
	
	showLogo();
	
	uartSetup();
	eState m_state = idle_state;
	m_state = StateMachineTable[m_state].StartHandler(m_state);
	
	while(TRUE)
	{
		int res = packetReceiving(&pktRcvdFlag, rx_buffer, RX_BUFFER_SIZE, 100000);
		
		switch (res)
		{
		case PACKET_RECEIVE_UART:
		{
			proto_s* msg_header = (proto_s*) rx_buffer;
			sprintf(&log[0][0], "Type: %s", TypeToString(msg_header->msg_type));
			sprintf(&log[1][0], "STATE: %s", toString(m_state));
			halBuiLcdUpdate(log[0], log[1]);
			m_state = StateMachineTable[m_state].ReceiveHandler(m_state, rx_buffer);
			break;
		}
		case PACKET_RECEIVE_RADIO:
		{
			static int radio_recv_cnt = 0;
			pktRcvdFlag = FALSE;
			m_state = StateMachineTable[m_state].RadioReceiveHandler(m_state, radioPktBuffer);
			DMAARM = DMAARM_CHANNEL0;
			RFST = STROBE_RX;
			sprintf(&log[0][0], "   Radio[%d] ", radio_recv_cnt++);
			sprintf(&log[1][0], "Packet receive");
			halBuiLcdUpdate(log[0], log[1]);
			break;
		}
		case PACKET_RECEIVE_TIMEOUT:
		{
			m_state = StateMachineTable[m_state].TimeoutHandler(m_state);
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
	
	static BOOL led_status = TRUE;
	if (led_status) LED3 = LED_ON;
	else LED3 = LED_OFF;
	led_status = !led_status;
	
	if (mode == RADIO_MODE_RX) {
		pktRcvdFlag = TRUE;
	}
	else {
		pktSentFlag = TRUE;
	}
}

/*==== END OF FILE ==========================================================*/
