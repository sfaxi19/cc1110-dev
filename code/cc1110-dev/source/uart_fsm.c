#include "../include/uart_fsm.h"
#include "../include/uart.h"
#include "../include/globals.h"

char log[2][16] = {0};

settings_s settings;
BOOL       is_settings_valid = FALSE;

const char* toString(eState state)
{
    switch(state)
    {
        case idle_state:       return "IDLE";
        case connecting_state: return "CONNECTING";
        case setup_state:      return "SETUP";
        case waitfor_state:    return "WAIT_FOR";
        case tx_active_state:  return "TX_ACTIVE";
        case rx_active_state:  return "RX_ACTIVE";
        case end_state:        return "END";
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
*                            Common
****************************************************************/
eState OnStart(eState state){
    return ChangeState(state, connecting_state);
}

eState Empty(eState state){
    return state;
}

eState EmptyTimeout(eState state){
    //strcpy(&log[0][0], "     Timeout    ");
    //strcpy(&log[1][0], "                ");
    //halBuiLcdUpdate(log[0], log[1]);
    return state;
}

eState EmptyRecv(eState state, uint8* data){
    return state;
}

/****************************************************************
*                          Connecting state
****************************************************************/
eState ConnectingOnBreak(eState state){
    return ChangeState(state, end_state);
}

eState ConnectingOnTimeout(eState state){
   /* if (state == waitfor_state){
        return ChangeState(state, connecting_state);
    } else{
        return ChangeState(state, waitfor_state);
    }*/
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
void SendSetupRsp(uint8* data){
    proto_s* msg = (proto_s*)data;
    msg->msg_type = SETUP_RSP;
    uart8Send(data, msg->data_size + sizeof(proto_s));
}

eState SetupOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case SETUP_REQ:
        settings_s* s = (settings_s*)(data + sizeof(proto_s));
        settings = *s;
        SendSetupRsp(data);
        return state;
    case SETUP_ACK:
        is_settings_valid = TRUE;
        radioSettingsApply(&settings);
        if (settings.MODE == RADIO_MODE_TX)
        {
            return ChangeState(state, tx_active_state);
        }
        else
        {
            return ChangeState(state, rx_active_state);
        }
    case SETUP_ERR:
        is_settings_valid = FALSE;
        return state;
    default: 
        return state;
    }
}
/****************************************************************
*                          WaitFor state
****************************************************************/
//eState WaitForOnBreak(eState state){
//    return ChangeState(state, end_state);
//}
//
//eState WaitForOnTimeout(eState state){
//    if (state == waitfor_state){
//        return ChangeState(state, connecting_state);
//    } else{
//        return ChangeState(state, waitfor_state);
//    }
//}
//
//eState WaitForOnRecv(eState state, uint8* data){
//    proto_s* header = (proto_s *) data;
//    switch(header->msg_type)
//    {
//    case SETUP_ACK:
//        return ChangeState(state, active_state);
//    default: 
//        return state;
//    }
//    return state;
//}

/****************************************************************
*                          TX Active state
****************************************************************/
void SendTxDataRsp(uint8* data){
    proto_s* msg = (proto_s*)data;
    msg->msg_type = DATA_RSP;
    uart8Send(data, msg->data_size + sizeof(proto_s));
}

eState TxActiveOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case DATA_ACK:
        radioSend();
        // Send data over radio!
        return state;
    case DATA_REQ:
        UINT8 i;
        for(i = 0; i < header->data_size; i++)
        {
            radioPktBuffer[i] = data[sizeof(proto_s) + i];
        }
        SendTxDataRsp(data);
        return state;
    default: 
        return state;
    }
    return state;
}
/****************************************************************
*                          RX Active state
****************************************************************/
void SendRxDataReq(uint8* data){
    proto_s msg;
    msg.msg_type = DATA_REQ;
    msg.data_size = PACKET_LENGTH + 2;
    uart8Send((uint8*)&msg, sizeof(proto_s));
    uart8Send(data, msg.data_size);
}

eState RxActiveOnRecv(eState state, uint8* data){
    proto_s* header = (proto_s *) data;
    switch(header->msg_type)
    {
    case DATA_RSP:
        return state;
    default: 
        return state;
    }
    return state;
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
    {waitfor_state,    Empty,     Empty,       EmptyTimeout,   EmptyRecv,         EmptyRecv},
    {tx_active_state,  Empty,     Empty,       EmptyTimeout,   TxActiveOnRecv,    EmptyRecv},
    {rx_active_state,  Empty,     Empty,       EmptyTimeout,   RxActiveOnRecv,    RxActiveOnRedioRecv},
    {end_state,        OnStart,   Empty,       EmptyTimeout,   EmptyRecv,         EmptyRecv}
};

