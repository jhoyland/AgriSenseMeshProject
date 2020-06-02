/*
 * Packet_Setup.c
 *
 *  Created: 5/28/2020 11:58:29 AM
 *  Author: Michael
 *  Functions for reading/writing to the information packages
 */ 
#include <avr/io.h>
#include "Packet_Setup.h"
#include "bitmanip.h"
#include "packet_specs.h"

void Set_Packet_Size(uint8_t* buff, uint8_t sz)
{
	buff[PK_COMMAND_HEADER + PK_SZ_PACKET] = sz;
}

void Set_Target_Node(uint8_t* buff, uint16_t target_node)
{
	word_to_bytes(&buff[PK_DEST_ADDR_HI],target_node);
}

void Set_Command(uint8_t* buff, uint16_t cmd_id, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4, uint8_t cmd5)
{
	word_to_bytes(& buff[PK_COMMAND_HEADER+PK_CMD_HI],cmd_id);
	buff[PK_COMMAND_HEADER+PK_CMD_DATA_0] = cmd2;
	buff[PK_COMMAND_HEADER+PK_CMD_DATA_1] = cmd3;
	buff[PK_COMMAND_HEADER+PK_CMD_DATA_2] = cmd4;
	buff[PK_COMMAND_HEADER+PK_CMD_DATA_3] = cmd5;
}
