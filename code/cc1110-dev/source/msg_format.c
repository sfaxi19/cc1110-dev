#include "../include/msg_format.h"

const char* TypeToString(eMsgType type){
  switch(type) {
    case WAKEUP:     return "WAKEUP";
    case WAKEUP_ACK: return "WAKEUP_ACK";
    case SETUP_REQ:  return "SETUP_REQ";
    case SETUP_RSP:  return "SETUP_RSP";
    case SETUP_ACK:  return "SETUP_ACK";
    case SETUP_ERR:  return "SETUP_ERR";
    case DATA_REQ:   return "DATA_REQ";
    case DATA_RSP:   return "DATA_RSP";
    case DATA_ACK:   return "DATA_ACK";
    case ERR:        return "ERR";
    default:     return "UNKNOWN";
  }
}