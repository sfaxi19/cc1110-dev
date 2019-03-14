#include "../include/ioCCxx10_bitdef.h"
#include "../include/uart_fsm.h"
#include "../include/uart.h"
#include "../include/globals.h"

static char log[2][16] = {0};

static settings_s settings;
static BOOL       is_settings_valid = FALSE;

const char* toString(eState state)
{
    switch(state)
    {
        case idle_state:       return "IDLE";
        case connecting_state: return "CONNECTING";
        case setup_state:      return "SETUP";
        case tx_active_state:  return "TX_ACTIVE";
        case rx_active_state:  return "RX_ACTIVE";
        default:               return "UNKNOWN";
    }
}

void printState(eState state)
{
    strcpy(&log[0][0], toString(state), 16);
    strcpy(&log[1][0], "                ");
    halBuiLcdUpdate(log[0], log[1]);
}

void printChangeState(eState stateCur, eState stateNext)
{
    strcpy(&log[0][0], toString(stateCur), 16);
    strcpy(&log[1][0], toString(stateNext), 16);
    halBuiLcdUpdate(log[0], log[1]);
}

eState ChangeState(eState prevState, eState curState){
    printChangeState(prevState, curState);
    return curState;
}

/****************************************************************
*                            Common(Idle)
****************************************************************/
eState OnStart(eState state){
    return ChangeState(state, connecting_state);
}

eState Empty(eState state){
    return state;
}

eState EmptyTimeout(eState state){
    return state;
}

eState EmptyRecv(eState state, uint8* data){
    return state;
}

/****************************************************************
*                          Connecting state
****************************************************************/
eState ConnectingOnBreak(eState state){
    return ChangeState(state, idle_state);
}

eState ConnectingOnTimeout(eState state){
    return state;
}

void SendWakeupAck(){
    proto_s msg;
    msg.msg_type = WAKEUP_ACK;
    msg.data_size = 0;
    uart8Send((uint8 *)&msg, sizeof(msg));
}

eState ConnectingOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case WAKEUP:
        SendWakeupAck();
        return ChangeState(state, setup_state);
    default: 
        return state;
    }
}
/****************************************************************
*                          Setup state
****************************************************************/
void SendSetupAck(){
	proto_s msg;
    msg.msg_type = SETUP_ACK;
    msg.data_size = 0;
    uart8Send((uint8 *)&msg, sizeof(msg));
}

eState SetupOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case SETUP_REQ:
		
        settings_s* s = (settings_s*)(data + sizeof(proto_s));
        settings = *s;
		
		is_settings_valid = TRUE;
        radioSettingsApply(&settings);
		
		SendSetupAck();
		
        if (settings.MODE == RADIO_MODE_TX) {
            return ChangeState(state, tx_active_state);
        } else {
            return ChangeState(state, rx_active_state);
        }
        return state;
    case SETUP_ERR:
        is_settings_valid = FALSE;
        return state;
	case WAKEUP:
		return ConnectingOnRecv(state, data);
    default: 
        return state;
    }
}

/****************************************************************
*                          TX Active state
****************************************************************/
void SendTxDataAck(){
	proto_s msg;
    msg.msg_type = DATA_ACK;
    msg.data_size = 0;
    uart8Send((uint8 *)&msg, sizeof(msg));
}

eState TxActiveOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case DATA_REQ:
		uint16 i;
		//   4 bytes     PACKET_LENGTH bytes        4 bytes       2 bytes
		// [ header ][         data           ][ TRANSMISSIONS][   CRC   ]
        for( i = 0; i < header->data_size - sizeof(uint32); i++)
        {
            radioPktBuffer[i] = data[sizeof(proto_s) + i];
        }
		uint32* transmissions = (uint32*)&data[sizeof(proto_s) + i];

        radioSending(*transmissions);
        SendTxDataAck(data);
        return state;
	case SETUP_REQ:
		return SetupOnRecv(state, data); 
	case WAKEUP:
		return ConnectingOnRecv(state, data);;
    default: 
        return state;
    }
}
/****************************************************************
*                          RX Active state
****************************************************************/
void SendRxDataReq(uint8* data){
	uint8 buffer[256];
    proto_s* msg = (proto_s*)buffer;
    msg->msg_type = DATA_REQ;
    msg->data_size = PACKET_LENGTH + 2;
	
	//sprintf(&log[0][0], "Type: %s", TypeToString(msg->msg_type));
	//sprintf(&log[1][0], "Size: %u", msg->data_size);
	//halBuiLcdUpdate(log[0], log[1]);
	//while(!halBuiButtonPushed());
	
	for (int i = 0; i < msg->data_size; i++)
	{
		buffer[sizeof(proto_s) + i] = data[i];
	}

    uart8Send(buffer, sizeof(proto_s) + msg->data_size);
}

eState RxActiveOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
	case WAKEUP:
		RFST = RFST_SNOP;                 // Switch radio to RX
		return ConnectingOnRecv(state, data);
	case SETUP_REQ:
		return SetupOnRecv(state, data); 
    default: 
        return state;
    }
}

eState RxActiveOnRedioRecv(eState state, uint8* data)
{
    SendRxDataReq(data);
    return state;
}

/****************************************************************
*                          FSM Table
****************************************************************/
StateMachine StateMachineTable [] =
{
//   State             OnStart    OnBreak      OnTimeout       OnRecv             OnRadioRecv
    {idle_state,       OnStart,   Empty,       EmptyTimeout,   EmptyRecv,         EmptyRecv},
    {connecting_state, Empty,     Empty,       EmptyTimeout,   ConnectingOnRecv,  EmptyRecv},
    {setup_state,      Empty,     Empty,       EmptyTimeout,   SetupOnRecv,       EmptyRecv},
    {tx_active_state,  Empty,     Empty,       EmptyTimeout,   TxActiveOnRecv,    EmptyRecv},
    {rx_active_state,  Empty,     Empty,       EmptyTimeout,   RxActiveOnRecv,    RxActiveOnRedioRecv},
};

