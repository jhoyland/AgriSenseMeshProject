/*
 * pktspec.h
 *
 * Created: 2019-02-08 2:00:56 PM
 *  Author: hoyla
 */ 


#ifndef PKTSPEC_H_
#define PKTSPEC_H_


#define PK_SZ_TXRX_BUFFER 40
#define PK_SZ_ERR_BUFFER 24

// Addressing header bytes
#define PK_SZ_ADDR_HEADER 8
#define PK_DEST_PANID_HI 0
#define PK_DEST_PANID_LO 1
#define PK_DEST_ADDR_HI 2
#define PK_DEST_ADDR_LO 3
#define PK_SRC_PANID_HI 4
#define PK_SRC_PANID_LO 5
#define PK_SRC_ADDR_HI 6	// column no
#define PK_SRC_ADDR_LO 7	// row no

#define PK_COMMAND_HEADER 8

// Command header bytes
#define PK_SZ_CMD_HEADER 8
#define PK_HOP_COUNT 0
#define PK_SZ_PACKET 1
#define PK_CMD_HI 2
#define PK_CMD_LO 3
#define PK_CMD_DATA_0 4
#define PK_CMD_DATA_1 5
#define PK_CMD_DATA_2 6
#define PK_CMD_DATA_3 7

// Body
#define PK_DATA_START 16

#define PK_DATUM_SIZE 3

#define PI_ADDR_HI 0x31
#define PI_ADDR_LO 0x42

#endif /* INCFILE1_H_ */