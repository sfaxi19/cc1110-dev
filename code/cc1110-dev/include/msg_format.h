#ifndef MSG_FORMAT_H
#define MSG_FORMAT_H

#include "../../Library/HAL/include/hal_defines.h"

typedef enum uint16
{
    WAKEUP = 1,
    WAKEUP_ACK,
	
    SETUP_REQ,
    SETUP_ACK,
    SETUP_ERR,
	
    DATA_REQ,
    DATA_ACK,
	
    ERR,
	NONE
} eMsgType;

const char* TypeToString(eMsgType type);

typedef struct {
    uint16 msg_type;
    uint16 data_size;
} proto_s;

    
typedef struct
{
  uint8 CC1110_SYNC1;
  uint8 CC1110_SYNC0;
  uint8 CC1110_PKTLEN;
  uint8 CC1110_PKTCTRL1;
  uint8 CC1110_PKTCTRL0;
  uint8 CC1110_ADDR;
  uint8 CC1110_CHANNR;
  uint8 CC1110_FSCTRL1;
  uint8 CC1110_FSCTRL0;
  uint8 CC1110_FREQ2;
  uint8 CC1110_FREQ1;
  uint8 CC1110_FREQ0;
  uint8 CC1110_MDMCFG4;
  uint8 CC1110_MDMCFG3;
  uint8 CC1110_MDMCFG2;
  uint8 CC1110_MDMCFG1;
  uint8 CC1110_MDMCFG0;
  uint8 CC1110_DEVIATN;
  uint8 CC1110_MCSM2;
  uint8 CC1110_MCSM1;
  uint8 CC1110_MCSM0;
  uint8 CC1110_FOCCFG;
  uint8 CC1110_BSCFG;
  uint8 CC1110_AGCCTRL2;
  uint8 CC1110_AGCCTRL1;
  uint8 CC1110_AGCCTRL0;
  uint8 CC1110_FREND1;
  uint8 CC1110_FREND0;
  uint8 CC1110_FSCAL3;
  uint8 CC1110_FSCAL2;
  uint8 CC1110_FSCAL1;
  uint8 CC1110_FSCAL0;
  uint8 CC1110_TEST2;
  uint8 CC1110_TEST1;
  uint8 CC1110_TEST0;
  uint8 CC1110_PA_TABLE7;
  uint8 CC1110_PA_TABLE6;
  uint8 CC1110_PA_TABLE5;
  uint8 CC1110_PA_TABLE4;
  uint8 CC1110_PA_TABLE3;
  uint8 CC1110_PA_TABLE2;
  uint8 CC1110_PA_TABLE1;
  uint8 CC1110_PA_TABLE0;
  uint8 CC1110_IOCFG2;
  uint8 CC1110_IOCFG1;
  uint8 CC1110_IOCFG0;
  uint8 CC1110_PARTNUM;
  uint8 CC1110_VERSION;
  uint8 CC1110_FREQEST;
  uint8 CC1110_LQI;
  uint8 CC1110_RSSI;
  uint8 CC1110_MARCSTATE;
  uint8 CC1110_PKTSTATUS;
  uint8 CC1110_VCO_VC_DAC;
  uint8 MODE;
  uint8 DUMMY_BYTE;
} settings_s;



#endif