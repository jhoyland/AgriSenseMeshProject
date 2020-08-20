/*
 * Packet_Setup.h
 *
 * Created: 5/28/2020 11:54:37 AM
 *  Author: Michael
 */ 


#ifndef PACKET_SETUP_H_
#define PACKET_SETUP_H_
#include <avr/io.h>

void clear_buffer(uint8_t*);
void Pk_Set_Dest_Panid(uint8_t*,uint16_t);
void Pk_Set_Src_Panid(uint8_t*,uint16_t);
void Pk_Set_Packet_Size(uint8_t*, uint8_t);
void Pk_Set_Target_Node(uint8_t*, uint16_t);
void Pk_Set_Final_Node(uint8_t*, uint16_t);
void Pk_Set_Command(uint8_t*, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);
void Pk_Set_Src_Node(uint8_t*, uint16_t);
void Pk_Add_Data(uint8_t*,uint16_t,uint8_t); //which bit of data to add
/*void Pk_Set_Data_Direction(uint8_t*,uint8_t);*/
void Pk_Set_Hop_Count(uint8_t*,uint8_t);


#endif /* PACKET_SETUP_H_ */