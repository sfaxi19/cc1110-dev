#include "../../Library/HAL/include/hal_main.h"
#include "../include/globals.h"
#include "../include/uart.h"
#include "../include/uart_fsm.h"
#include "../include/msg_format.h"

#define RX_BUFFER_SIZE 1024
uint8 rx_buffer[RX_BUFFER_SIZE];

BYTE PACKET_LENGTH;
uint32 TRANSMISSIONS = 1;

void radioSend()
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
    UINT8 i;
    UINT32 burstSize;                   // Number of packets to burst
    UINT32 seqNum;                      // Sequence number for TX packet

    // Initialize LED1
    INIT_LED1();
    INIT_LED3();

    // Choose the crystal oscillator as the system clock
    halPowerClkMgmtSetMainClkSrc(CRYSTAL);

    // Initialize the LCD
    halBuiInitLcd();

    // Show Chipcon logo and chip name + revision until S1 is pushed
    showLogo();
    //while(!halBuiButtonPushed());
       
    // Check that chip version is not too old or too new to be supported by
    // the register settings used in this software.
    //checkChipVersion();
    uartSetup();
    
    // Select frequency and data rate from LCD menu, then configure the radio
    //radioConfigure(selectDataRate(), selectRadioFrequency());

    // Select from LCD menu either transmitter or receiver mode
    //mode = selectMode();
    //mode = RADIO_MODE_TX;
    eState m_state = idle_state;
    m_state = StateMachineTable[m_state].StartHandler(m_state);

    //halBuiLcdUpdate("UART FSM", "INIT");
    char log[2][16];
    while(TRUE) 
    {
        int res = packetReceiver(&pktRcvdFlag, rx_buffer, RX_BUFFER_SIZE, 100000);

        if (res == PACKET_RECEIVE_UART)
        {
            proto_s* msg_header = (proto_s*)rx_buffer;
            sprintf(&log[0][0], "Type: %s", TypeToString(msg_header->msg_type));
            sprintf(&log[1][0], "Size: %u", msg_header->data_size);
            halBuiLcdUpdate(log[0], log[1]);
            m_state = StateMachineTable[m_state].ReceiveHandler(m_state, rx_buffer);
        }
        else if (res == PACKET_RECEIVE_RADIO)
        {
            static int radio_recv_cnt = 0;
            pktRcvdFlag = FALSE;
            m_state = StateMachineTable[m_state].RadioReceiveHandler(m_state, radioPktBuffer);
            // We don't need our packet buffer anymore, prepare for the next packet
            DMAARM = DMAARM_CHANNEL0;
            RFST = STROBE_RX;
            sprintf(&log[0][0], "   Radio[%d] ", radio_recv_cnt++);
            sprintf(&log[1][0], "Packet receive");
            halBuiLcdUpdate(log[0], log[1]);
        }
        else if (res == PACKET_RECEIVE_TIMEOUT)
        {
            m_state = StateMachineTable[m_state].TimeoutHandler(m_state);
        }
    }
    /*  
    if (mode == RADIO_MODE_TX) {
        // Set up the DMA to move packet data from buffer to radio
        dmaRadioSetup(RADIO_MODE_TX);

        // Configure interrupt for every time a packet is sent
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

        // Construct the packet to be transmitted in buffer
        radioPktBuffer[0] = PACKET_LENGTH;                  // Length byte
        radioPktBuffer[1] = (BYTE) (NETWORK_ID_KEY>>8);     // Network identifier
        radioPktBuffer[2] = (BYTE) NETWORK_ID_KEY;
        // radioPktBuffer[3:6] = 4 byte packet sequence number, written later
        // Fill rest of payload with dummy data. Radio is using data whitening.
        for (i = 7; i <= PACKET_LENGTH; i++) {
            radioPktBuffer[i] = 0xCC;
        }

        // Select from LCD menu the packet burst size
        burstSize = selectBurstSize();

        while (TRUE) {

            // Wait for S1 button press before starting
            halBuiLcdUpdate("Press S1 to", "start burst ...");
            while (!halBuiButtonPushed());

            halBuiLcdUpdate("Pkts transmit'd:", "");

            LED1 = LED_ON;

            // Transmit the packet burst
            for (seqNum = 1; seqNum <= burstSize; seqNum++) {

                // Set correct sequence number to packet
                pktSetSeqNum(seqNum);

                // Send the packet
                DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
                RFST = STROBE_TX;           // Switch radio to TX

                // Wait until the radio transfer is completed,
                // and then reset pktSentFlag
                while(!pktSentFlag);
                pktSentFlag = FALSE;

                // Update LCD with the packet counter
                lcdWriteSeqNum(seqNum);

                // Wait approx. 3 ms to let the receiver perform its
                // tasks between each packet
                halWait(3);

                // Abort burst if button S1 is pushed.
                if (halBuiButtonPushed() == TRUE) {
                    break;
                }
                //uart8Send(radioPktBuffer, PACKET_LENGTH+3);
            }

            LED1 = LED_OFF;                    
        }
    }
    else if (mode == RADIO_MODE_RX) {

        // Set up the DMA to move packet data from radio to buffer
        dmaRadioSetup(RADIO_MODE_RX);

        // Configure interrupt for every received packet
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

        halBuiLcdUpdate("Ready to", "receive");

        // Start receiving
        DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
        RFST   = STROBE_RX;                 // Switch radio to RX

        // Do not update the LCD until the first packet is received
        while (!pktRcvdFlag);
        halBuiLcdUpdate("PER:   0.0 %", "RSSI:      dBm");

        while (TRUE) {
            // Poll for incoming packet delivered by radio + dma
            if (pktRcvdFlag) {
                pktRcvdFlag = FALSE;
                //uart8Send(radioPktBuffer, PACKET_LENGTH+3);
                // Check if the received packet is a valid PER test packet
                if (pktCheckValidity()) {

                    // A PER test packet has been received, hence the statistics
                    // on the LCD must be updated.
                    updateLcd = TRUE;
                    LED1 = LED_ON;      // Turn on LED to indicate PER test link

                    // Subtract old RSSI value from sum
                    perRssiSum -= perRssiBuf[perRssiBufCounter];
                    // Store new RSSI value in ring buffer, will add it to sum later
                    perRssiBuf[perRssiBufCounter] = convertRssiByte(radioPktBuffer[PACKET_LENGTH+1]);
                }

                // We don't need our packet buffer anymore, prepare for the next packet
                DMAARM = DMAARM_CHANNEL0;
                RFST = STROBE_RX;

                // Update the LCD only if necessary
                if (updateLcd) {

                    // Calculate PER of received packets in unit per 1000
                    // and print to LCD
                    lcdWritePer();

                    // Add the new RSSI value to sum. Calculate and print
                    // average RSSI to LCD
                    perRssiSum += perRssiBuf[perRssiBufCounter];
                    lcdWriteRssi(perRssiSum);
                    if (++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) {
                        perRssiBufCounter = 0;      // Wrap ring buffer counter
                    }

                    // Update blinking cursor to indicate there is activity
                    // (LED toggles according to bit5 of counter)
                    halBuiLcdUpdateChar(LINE1, 15,
                                        blinkCursor[(++blinkCursorIdx & 0x20)
                                            >> 5]);

                    updateLcd = FALSE;
                    LED1 = LED_OFF;
                }
            }
        }
    }*/
}


/*==== PRIVATE FUNCTIONS =====================================================*/



/*==== INTERRUPT SERVICE ROUTINES ============================================*/

/******************************************************************************
* @fn  rf_IRQ
*
* @brief
*      The only interrupt flag which throws this interrupt is the IRQ_DONE interrupt.
*      So this is the code which runs after a packet has been received or
*      transmitted.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
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
