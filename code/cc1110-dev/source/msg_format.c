#include "../include/msg_format.h"

const char* TypeToString(eMsgType type){
  switch(type) {
    case WAKEUP:     return "WAKEUP";
    case WAKEUP_ACK: return "WAKEUP_ACK";
	
    case SETUP_REQ:  return "SETUP_REQ";
    case SETUP_ACK:  return "SETUP_ACK";
    case SETUP_ERR:  return "SETUP_ERR";
	
    case DATA_REQ:   return "DATA_REQ";
    case DATA_ACK:   return "DATA_ACK";
	
    case ERR:        return "ERR";
	case NONE:       return "NONE";  
    default:     return "UNKNOWN";
  }
}