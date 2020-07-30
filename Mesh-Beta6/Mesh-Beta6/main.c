/*
 * Mesh-Beta6.c
 *
 * Created: 6/17/2020 12:51:09 PM
 * Author : Michael
 * Attempting to include network setup and other functionalities to the relay system
 */
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include "PinDefs_2_18_2020.h"
#include "command_specs_2_20_2020.h"
#include "tinyspi.h"
#include "mrf24j.h"
#include "bitmanip.h"
#include "packet_specs.h"
#include "blinkin.h"
#include "Packet_Setup.h"
#include "Network_functions.h"


#define THIS_DEVICE 0x0002 //change this to program each node's ID
#define ASMP_PANID 0xCAFE //PANID for the network

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER]; //holds data to be sent
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER]; //holds received data

//TODO: Global variables tend to be a bad idea. I should work on making it so these are properly encapsulated

uint16_t node_list[5] = {0x0001,0x0002,0x0003,0x0004,0x0005}; //list of the node ID's
uint16_t observed_nodes[5]; //holds the list of nodes that are known to be on the network. This list will build as the message is relayed back to the pi from the end node
uint8_t target_index; //index of the node list to be searched. Global variable??
uint8_t node_count = 5; //holds the number of nodes that are in the network (eventually read/write from file) changed to 3 for simplicity
uint8_t neighbor_count; //holds the number of neighbors a node has (should max be 2)
//uint8_t target_node; //holds the number of nodes searched in a network
uint8_t network_status; //status of the network: complete or incomplete (could turn to bool)
uint8_t node_status; //holds the current node status, setting up or idle right now
uint16_t pi_address = 0x3142; //address of the base raspberry pi
bool Network_Set; //candidate for a global variable I'll use

struct neighbor
{
	uint16_t id;
	uint8_t sensors; //a bitmask for the sensors available??
	uint8_t upstairs_id; //this node's "upstream" neighbor
	uint8_t downstairs_id; //this node's "downstream" neighbor
};


uint8_t running_status = 0;
struct neighbor downstairs_neighbor; //down the line towards the pi
struct neighbor upstairs_neighbor; //up towards the end of the line



void handle_rx()
{
	running_status |= (1<<RU_RX_HANDLE);  //MUTEX
	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()); //makes a copy of the rx data to a buffer
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	COMMAND_HANDLER(recieved_data_buffer); //puts the received data buffer into the command handler
	running_status &= ~(1<<RU_RX_HANDLE);
	
}

void handle_tx()
{
	//doesn't really need to do anything upon sending a packet... yet...?
	//PORTD |= (1<<RED_LIGHT);
}


void COMMAND_HANDLER(uint8_t* message) //Looks at the command bits and decides what to do with the given command
{	
	switch(bytes_to_word(&message[PK_COMMAND_HEADER+PK_CMD_HI])) //look at the command given to the node
	{
		case CMD_PING: //node is being pinged
		ping_handler(message);
		break;
		case CMD_ECHO: //node pinged responded to a ping
		echo_handler(message);
		break;
		case CMD_DATA:
		//collect_data(message); //collect data from the required sensors TODO: Implement ADC collection
		break;
		case CMD_SETUP:
		setup_network(message);//node is now in network setup mode
		break;
		case CMD_PROBE_NEIGHBORS: //node has received a request to return the number (and ID?) of its neighbors
		confirm_neighbor(message);
		break;
		case CMD_NEIGHBOR_COUNT: //has received a response containing the number of neighbors (during a setup routine)
		set_upstairs_neighbor(message);
		break;
		case CMD_NETWORK_COMPLETE:
		send_downstream(message);//relay the message down that the network is complete. Should go all the way to the pi.
		break;
		case CMD_TO_PI:
		send_directly_to_pi(message);
		default:
		//command not recognized
		break;	
	}
	
}


void ping_handler(uint8_t* message) //this loop will only enter upon receiving the message CMD_PING
{ 
	if(Network_Set)
	{
		//this is to determine if the chain is still present
		if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 0)
		{ //first "Accessory bit" is 0: This is just a boring ping									  
			Pk_Set_Command(message,CMD_ECHO,0,0,0,0); //respond with "echo" in the command location
			send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),message); //send it back to the originator
		}
		if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 1)
		{ 
			//first accessory bit is 1: functionality not defined yet
		}
	}
	else
	{
		//send a message stating that the network has to be set up before you can ping??
	}
}
void echo_handler(uint8_t* message) //haven't decided what I want to do with this
{
	
} 

void setup_network(uint8_t* message)
{	
	//this function should only enter once, when the node is told to setup the network initially
	//TODO: Have a case where the network is already setup
	node_status = SETTING_UP; //flag for set up routine
	Pk_Set_Command(transmit_data_buffer,0,node_status,0,0,0);
	send_directly_to_pi(transmit_data_buffer);
	neighbor_count = 0; //default it to 0 -> when routine is entered
	set_downstairs_neighbor(message); //set the downstairs node for this (node that messages will be relayed to) as the person who requested this
	probe_neighbor_status(); //start by searching node 0x0001 -> must wait for a response
	wait_for_response(message);
}

void continue_setup(uint8_t target) //tells the next node to do the setup routine
{
	Pk_Set_Command(transmit_data_buffer,CMD_SETUP,0,0,0,0);
	send_message(target,transmit_data_buffer);
	node_status = IDLE;
}

void set_downstairs_neighbor(uint8_t* message)
{
	downstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
	if(downstairs_neighbor.id == 0x0001) {BLINK(LIGHT_PORT,RED_LIGHT); BLINK(LIGHT_PORT,YELLOW_LIGHT); BLINK(LIGHT_PORT,GREEN_LIGHT);}
	if(downstairs_neighbor.id == pi_address)  {BLINK(LIGHT_PORT,RED_LIGHT); BLINK(LIGHT_PORT,YELLOW_LIGHT); BLINK(LIGHT_PORT,GREEN_LIGHT);}
	++neighbor_count;
}

void set_upstairs_neighbor(uint8_t* message) //this function may prove to be my undoing. Tread carefully
{
	//if the node has a downstairs neighbor, it needs an upstairs neighbor unless it is the last node
	//the node needs to ask the others in the field if they have a neighbor, and if not, choose that one as an upstairs neighbor
	//if a node received this request, and it has 1 neighbor, it should tell the requester to ask someone else
	//the number of neighbors will be loaded into PK_CMD_DATA_0
	if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 0) //this holds the number of neighbors the probed node has
	{
		upstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
		++neighbor_count; //increase the amount of neighbors. Generally speaking, this should always result in 2.
		PORTD &= ~(1<<GREEN_LIGHT);
		if(neighbor_count >= 2) Network_Set = true;
		if(Network_Set) PORTD |= (1<<RED_LIGHT);
		node_status = IDLE;
		//if(neighbor_count >2 ) PORTD|= (1<<YELLOW_LIGHT); //seeing if I'm over-counting neighbors
		continue_setup(upstairs_neighbor.id);
	}
	else 
	{	
		//PORTD |= (1<<GREEN_LIGHT);
		++target_index; //increment through the list
		if(target_index > node_count) //we have exceeded the number of nodes available without finding a free node
		{
			//this node is the last node, relay that info back to the pi
			upstairs_neighbor.id = 0x9999; //no upstairs neighbor
			node_status = IDLE;
			Network_Set = true;
			if(Network_Set) PORTD |= (1<<RED_LIGHT);
			Pk_Add_Data(transmit_data_buffer,0x9999); //look for this message
			confirm_network_complete(); //uses the source address	
		}
		/*else
		{
			wait_for_response(message);
		}*/ //is this needed?
	}
}


void wait_for_response(uint8_t * message)
{
	//this function waits for a response with a timer. If the time has passed and a message has not been received, the next node is searched
	//this is needed because the first message a node receives upon a reboot is thrown out
	//also controls for possibly randomly dropped messages
	node_status = WAITING_FOR_RESPONSE; //change the node status
	PORTD |= (1<<GREEN_LIGHT);
	uint8_t overflow_counter = 0;
	uint8_t repeat_counter = 0;
	PRR |= (0<<PRTWI); //write 1 to the PRTWI bit on PRR (power reduction register) to ensure clock is running: Should default to 0. This is just in case?
	TCCR0A |= (1<<CTC0); //set CTC bit (clear timer compare)
	TCCR0A |= (1<<CS02) | (1<< CS00); //clock select 1024 prescaler
	TCNT0 = 0; //set the 8 bit timer to 0
	while(node_status == WAITING_FOR_RESPONSE)
	{
		//mrf_check_flags(&handle_rx,&handle_tx);		
		while((TIFR0 & 0x01) == 0) mrf_check_flags(&handle_rx,&handle_tx); //repeated action while the overflow hasn't occurred
		TCNT0 = 0; //write 0 to the counter on overflow
		TIFR0 = 0x01; //timer interrupt flag register cleared by writing 1. (this is super important, by the way)
		++overflow_counter; //increment overflow counter
		//So what happens now? If I get a message that I'm waiting for (the command) this function needs to exit.
		//if I get something else, or nothing at all, the function needs to continue
		if(overflow_counter >= 10 ) //by calculator 39 ~= 10 seconds
			{
				send_directly_to_pi(transmit_data_buffer); //is this function executing?
				probe_neighbor_status();
				++repeat_counter;
				overflow_counter = 0;
				if(repeat_counter >= 2)
					{
						BLINK(LIGHT_PORT,RED_LIGHT);
						++target_index;
						//send_message(node_list[target_index],transmit_data_buffer);
						probe_neighbor_status();
						repeat_counter = 0;
						overflow_counter = 0;
					}
				}
		}
	

}

/*void collect_data(uint8_t* message)
{
	//depending on the sensors requested, add data	
}*/

void send_directly_to_pi(uint8_t* buff)
{
	send_message(pi_address,buff);
}

void send_downstream(uint8_t* buff) //is this function entering? 29/7/2020
{
	if(Network_Set)
	{
		PORTD |= (1<<YELLOW_LIGHT);
		//Pk_Set_Command(buff,CMD_TO_PI,0,0,0x99,0x99);
		Pk_Add_Data(transmit_data_buffer,0x7777);
		send_message(downstairs_neighbor.id,buff);
	}
	else
	{
		Pk_Set_Command(transmit_data_buffer,CMD_TO_PI,0,0,0,0);
		Pk_Add_Data(transmit_data_buffer,0x8888);
		PORTD |= (1<<GREEN_LIGHT);
		send_message(pi_address,buff);
	}

}

void confirm_network_complete()
{
	Pk_Set_Command(transmit_data_buffer,CMD_NETWORK_COMPLETE,0,0,0,0);
	send_message(downstairs_neighbor.id,transmit_data_buffer);
	if(downstairs_neighbor.id == 0x0001) PORTD |= (1<<GREEN_LIGHT);	
}
void probe_neighbor_status(/*uint8_t * target_index*/) 
{
	//BLINK(LIGHT_PORT,RED_LIGHT);
	//clear_buffer(transmit_data_buffer);
	if(node_list[target_index] == THIS_DEVICE) ++target_index; //to avoid possibly sending a message to itself
	Pk_Set_Command(transmit_data_buffer,CMD_PROBE_NEIGHBORS,0,0,0,0);
	//Pk_Add_Data(transmit_data_buffer,0x5555);
	send_directly_to_pi(transmit_data_buffer); //for troubleshooting: Is this causing a glitch?
	//28/7/2020 I am not receieving a slough of these at the Pi. Why??
	send_message(node_list[target_index],transmit_data_buffer);
}
void confirm_neighbor(uint8_t* message) //responds to a request for the number of neighbors
{
	//memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER); //maybe not needed
	Pk_Set_Command(transmit_data_buffer,CMD_NEIGHBOR_COUNT,neighbor_count,0,0,0);
	send_directly_to_pi(transmit_data_buffer); //for troubleshooting
	//28/7/2020 not receiving a slough of these at the Pi. Why??
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]), transmit_data_buffer); //return this to the requesterr
}

void send_message(uint16_t target, uint8_t* buff)
{
	//memset(buff,0,PK_SZ_TXRX_BUFFER);
	BLINK(LIGHT_PORT,YELLOW_LIGHT);
	Pk_Set_Packet_Size(buff,PK_SZ_TXRX_BUFFER); //this should be a function that finds the packet size maybe?
	Pk_Set_Target_Node(buff,target);
	Pk_Set_Src_Node(buff,THIS_DEVICE);
	mrf_send16(target,buff,PK_SZ_TXRX_BUFFER);
}

ISR(INT0_vect) //for when the MRF interrupts (sending or receiving a message)
{
	running_status |= (1<<RU_INTERRUPT);
	mrf_interrupt_handler();
	running_status &= ~(1<<RU_INTERRUPT);
}



void setup()
{
	//LED setup
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1 << YELLOW_LIGHT);
	DDRD |= (1 << GREEN_LIGHT);
	
	//SPI port setup
	PORTB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS) | (1<<MRF_CS) ; //set these ports to high (required)
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS) | (1<<MRF_CS) ;  //set these to output
	DDRB &= ~(1<<SPI_MISO);	//master in slave out, input on attiny
	
	spi_setup();
	mrf_reset();
	mrf_init();
	
	mrf_set_pan(ASMP_PANID); //set PANID
	mrf_address16_write(THIS_DEVICE); //set device address -> 16 bit addressing
	sei(); //starts interrupts, essential to let chip know message is being handled
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
	
	memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset, possibly not needed
	memset(recieved_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset
	
	Network_Set = false; //default to network setup: No down or up neighbor
	node_status = IDLE; //default to idle
	neighbor_count = 0; //setup resets these values to 0
	target_index = 0;
	
	BLINK(LIGHT_PORT,GREEN_LIGHT);
}


int main(void)
{
    setup();
    while (1) 
    {
		while(node_status == IDLE) mrf_check_flags(&handle_rx, &handle_tx); //check for interrupts
    }
}

