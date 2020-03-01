#ifndef __CMD_SPEC_H
#define __CMD_SPEC_H


// Status bits

#define ST_PORTS 0
#define ST_SPI 1
#define ST_MRF 2
#define ST_BUFFERS 3
#define ST_INTERRUPTS 4
#define ST_POWER 5
#define ST_SENSORS 6
#define ST_OPTION 7

#define RU_CMD_EXEC 0
#define RU_DATA_COLLECT 1
#define RU_TX_HANDLE 2
#define RU_RX_HANDLE 3
#define RU_INTERRUPT 4
#define RU_SETUP 5
#define RU_RUNNING 6
#define RU_SLEEP 7

#define CMD_DATA 0x4441						// 'DA' Request data: command arguments include a unique request id and a bitmask specifying which channels to read 
#define CMD_PING 0x4552						// Ping:  
#define CMD_ECHO 0x4543
#define CMD_GET_PARAMETER 0x5047
#define CMD_SET_PARAMETER 0x5053
#define CMD_QSTATUS 0x5153
#define CMD_NODE_TEST 0x4A4A
#define CMD_PROBE_NEIGHBORS
#define CMD_SET_ADDRESS_LIST
#define CMD_GET_ADDRESS_LIST
#define CMD_FORWARD_TO
#define CMD_TRACE
#define CMD_NODE_ERROR 0x5858

#define NP_ADC_ACTIVE_CH 0
#define NP_ADC_N_SAMPLES 256

#define ERR_UNRECOGNIZED_COMMAND 0x0001
#define ERR_SETUP_FAILED	0x0002

#define ADC_N_CHANNELS 8

#endif