#ifndef __BASENODE_H
#define __BASENODE_H
#include <stdint.h>

void request_data(uint16_t target_node, uint16_t request_id);

void request_ping(uint16_t target_node, uint16_t request_id, uint8_t d);

// This is called by the interrupt handler if new data is received

void handle_rx() ;
// This is called by the interrupt handler when transmit has completed and the receiver has acknowledged
// The acknowledgement is automatic however, note it is the actual receiver of this message which sends
// acknowledge - not the final destination node. In this case all it tells us is if the BASE_NODE of the 
// network has received the message

void handle_tx() ;

void setup() ;

void loop() ;

#endif
