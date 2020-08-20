/*
 * network_setup_functions.c
 *
 * Created: 8/13/2020 12:15:00 PM
 *  Author: Michael
 */ 
#include "network_setup_functions.h"
#include "command_specs_2_20_2020.h"
#include <stdio.h>
void setup_network(uint8_t* buff, neighbor *upstairs_neighbor ,neighbor *downstairs_neighbor, uint8_t* node_status, uint8_t* target_index,uint8_t* neighbor_count,uint8_t* node_list)
{
	//this function should only enter once, when the node is told to setup the network initially
	//TODO: Have a case where the network is already setup
	*node_status = SETTING_UP; //flag for set up routine
	*neighbor_count = 0; //default it to 0 -> when setup routine is entered
	target_index = 0; //default
	*upstairs_neighbor->id = 0x0000; //default
	*downstairs_neighbor->id = 0x0000; //default
	set_downstairs_neighbor(buff); //set the downstairs node for this (node that messages will be relayed to) as the person who requested this
	probe_neighbor_status(node_list[target_index]); //start by searching node 0x0001 -> must wait for a response
	wait_for_response(buff);
}