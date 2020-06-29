/*
 * Network_functions.h
 *
 * Created: 6/26/2020 1:29:03 PM
 *  Author: Michael
 * Functions for setting up the network and sending messages
 */ 

void setup_network(uint8_t*); 
void continue_setup(uint8_t);
void ping_handler(uint8_t*);
void set_downstairs_neighbor(uint8_t*);
void set_upstairs_neighbor(uint8_t*);
void confirm_network_complete(uint8_t*);
void probe_neighbor_status(uint16_t);
void confirm_neighbor(uint8_t*);
void echo_handler(uint8_t*);
void send_message(uint16_t,uint8_t*);
