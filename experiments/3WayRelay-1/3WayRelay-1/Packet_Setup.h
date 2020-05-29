/*
 * Packet_Setup.h
 *
 * Created: 5/28/2020 11:54:37 AM
 *  Author: Michael
 */ 


#ifndef PACKET_SETUP_H_
#define PACKET_SETUP_H_
#include <avr/io.h>

void Set_Packet_Size(uint8_t*, uint8_t);
void Set_Target_Node(uint8_t*, uint16_t);
void Set_Command(uint8_t*, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);



#endif /* PACKET_SETUP_H_ */