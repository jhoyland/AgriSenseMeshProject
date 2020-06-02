#ifndef __BASENODE_H
#define __BASENODE_H
#include <stdint.h>



void set_packet_header(uint8_t* b);
void set_request_id(uint8_t* b);
void set_target_node(uint8_t* b,uint16_t target_node);
void set_command(uint8_t* b,uint16_t cmd_id, uint8_t cmd2, uint8_t cmd3);
void set_packet_size(uint8_t* b,uint8_t sz);

void send_command(uint8_t* b);

/* Convenience functions for extracting data from message*/

uint16_t get_command(uint8_t* msg);
uint8_t get_command_data(uint8_t* msg, uint8_t n);
uint16_t get_source_node_address(uint8_t* msg);
uint8_t get_message_size(uint8_t* msg);
uint8_t get_data_size(uint8_t* msg);
void get_data_array(uint8_t* msg, uint8_t n, uint16_t * out);
void get_data(uint8_t *msg, uint8_t n, uint16_t* d1, uint16_t* d2);
uint8_t get_hop_count(uint8_t* msg);

/* Form requests for specific commands */


void request_data(uint16_t target_node);
void request_ping(uint16_t target_node, uint8_t d);

/* Process the next message in the queue */

void process_next_node_message();
void process_node_data();
void process_node_ping_echo();
void print_message(uint8_t * msg);
void print_data(uint8_t * msg);

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
