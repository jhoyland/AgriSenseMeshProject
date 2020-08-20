/*
 * network_setup_functions.h
 *
 * Created: 8/13/2020 12:15:22 PM
 *  Author: Michael
 */ 
#include <stdio.h>
void setup_network(uint8_t*, struct neighbor*, struct neighbor*, uint8_t*, uint8_t*,uint8_t*);
void continue_setup(uint8_t);
void set_downstairs_neighbor(uint8_t*);
void set_upstairs_neighbor(uint8_t*);
void wait_for_response(uint8_t*);
void confirm_network_complete();
void set_last_node(uint8_t*);
void probe_neighbor_status(uint16_t);
void confirm_neighbor(uint8_t*);