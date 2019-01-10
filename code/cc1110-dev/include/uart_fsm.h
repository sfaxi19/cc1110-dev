#ifndef UART_FSM_H
#define UART_FSM_H

#include "../../HAL/include/hal_main.h"
#include "msg_format.h"

typedef enum {
    idle_state,
    connecting_state,
    setup_state,
    waitfor_state,
    tx_active_state,
    rx_active_state,
    end_state
} eState;

typedef eState (*eventHandler)(eState);

typedef eState (*eventRecvHandler)(eState, uint8* data);

const char* toString(eState state);

typedef struct {
    eState state;
    eventHandler StartHandler;
    eventHandler BreakHandler;
    eventHandler TimeoutHandler;
    eventRecvHandler ReceiveHandler;
    eventRecvHandler RadioReceiveHandler;

}StateMachine;

extern StateMachine StateMachineTable [];


#endif