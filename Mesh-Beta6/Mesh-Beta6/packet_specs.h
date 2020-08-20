/*
 * packet_specs.h
 *
 * Created: 2/25/2020 12:14:26 PM
 *  Author: Michael Hilborn, James Hoyland
 */ 
#ifndef PACKET_SPECS__H
#define PACKET_SPECS__H

#define PK_SZ_TXRX_BUFFER 40
#define PK_SZ_TX_BUFFER aMaxPHYPacketSize
#define PK_SZ_RX_BUFFER aMaxPHYPacketSize
#define PK_SZ_ERR_BUFFER 24

// Addressing header bytes:
#define PK_SZ_ADDR_HEADER 4  //8 formerly
//#define PK_DEST_PANID_HI 0 //not required
//#define PK_DEST_PANID_LO 1 //not required
#define PK_DEST_ADDR_HI 0
#define PK_DEST_ADDR_LO 1
#define PK_FINAL_ADDR_HI 2
#define PK_FINAL_ADDR_LO 3
//#define PK_SRC_PANID_HI 4 
//#define PK_SRC_PANID_LO 5 //not required
#define PK_SRC_ADDR_HI 4
#define PK_SRC_ADDR_LO 5

#define PK_COMMAND_HEADER 6 //start of the command header

// Command header bytes
#define PK_SZ_CMD_HEADER 8
#define PK_HOP_COUNT 0				// Hop count is used to count number of nodes the message is passed over -6
#define PK_SZ_PACKET 1				// Size of the entire message including the header. Required?			-7
#define PK_CMD_HI 2					// MSB of the command word -8
#define PK_CMD_LO 3					// LSB of the command word -9
#define PK_CMD_DATA_0 4				// Command data bytes: -10
#define PK_CMD_DATA_1 5				//     These four bytes are used for command specific arguments. -11
#define PK_CMD_DATA_2 6				//	   All four bytes must be sent even if not needed for the specific message -12
#define PK_CMD_DATA_3 7				// -13
#define PK_ADC_CHANNEL 8			//specifies the channel to get ADC data from -14
// Body
#define PK_DATA_START 15			// Actual data starts here

#define PK_DATUM_SIZE 3				// Size of individual data item. Three bytes are used to pack 2 x 12 bit values representing mean and sd.

#endif