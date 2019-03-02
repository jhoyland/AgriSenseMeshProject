#ifndef __CMD_SPEC_H
#define __CMD_SPEC_H


// Status bits

#define ST_PORTS 1
#define ST_SPI 2
#define ST_MRF 3
#define ST_BUFFERS 4
#define ST_INTERRUPTS 5
#define ST_POWER 6
#define ST_SENSORS 7
#define ST_OPTION 8

#define RU_CMD_EXEC 1
#define RU_DATA_COLLECT 2
#define RU_TX_HANDLE 3
#define RU_RX_HANDLE 4
#define RU_INTERRUPT 5
#define RU_SETUP 6



#define CMD_DATA 0x4441
#define CMD_PING 0x4552
#define CMD_ECHO 0x4543
#define CMD_GET_PARAMETER 0x5047
#define CMD_SET_PARAMETER 0x5053
#define CMD_QSTATUS 0x5153

#define ERR_UNRECOGNIZED_COMMAND 0x0001

#endif