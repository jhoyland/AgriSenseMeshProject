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


#define THIS_DEVICE 0x0001 //change this to program each node's ID
#define ASMP_PANID 0xCAFE //PANID for the network

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER]; //holds data to be sent
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER]; //holds received data

//TODO: Global variables tend to be a bad idea. I should work on making it so these are properly encapsulated

uint16_t node_list[5] = {0x0001,0x0002,0x0003,0x0004,0x0005}; //list of the node ID's
uint16_t observed_nodes[5]; //holds the list of nodes that are known to be on the network. This list will build as the message is relayed back to the pi from the end node
uint8_t target_index = 0; //index of the node list to be searched
uint8_t node_count = 5; //holds the number of nodes that are in the network (eventually read/write from file)
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
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	running_status |= (1<<RU_RX_HANDLE);  //MUTEX
	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()); //makes a copy of the rx data to a buffer
	//check the addressing bit to determine what should be done
	//if(bytes_to_word(&recieved_data_buffer[PK_DEST_ADDR_HI]) == THIS_DEVICE) //a message specifically for this node
	//{
		BLINK(LIGHT_PORT,GREEN_LIGHT); //INFINITE LOOP HERE (Don't know why)
		COMMAND_HANDLER(recieved_data_buffer); //puts the received data buffer into this
	//}
	//else
	//{
		//this message isn't for this node, do nothing. This should never happen
	//}
	running_status &= ~(1<<RU_RX_HANDLE);
	
}

void handle_tx()
{
	//doesn't really need to do anything upon sending a packet... yet...?
}


void COMMAND_HANDLER(uint8_t* message) //Looks at the command bits and decides what to do with the given command
{	//BLINK(LIGHT_PORT,GREEN_LIGHT);
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
		setup_network(/*message*/);//node is now in network setup mode
		break;
		case CMD_PROBE_NEIGHBORS: //node has received a request to return the number (and ID?) of its neighbors
		confirm_neighbor(message);
		break;
		case CMD_NEIGHBOR_COUNT: //has received a response containing the number of neighbors (during a setup routine)
		set_upstairs_neighbor(message);
		break;
		case CMD_NETWORK_COMPLETE:
		//relay the message down that the network is complete. Should go all the way to the pi.
		break;
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
			Pk_Set_Command(message,CMD_ECHO,0,0,0); //respond with "echo" in the command location
			send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),message); //send it back to the originator
		}
		if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 1)
		{ 
			//first accessory bit is 1: functionality not defined yet
		}
	}
}
void echo_handler(uint8_t* message) //haven't decided what I want to do with this
{
	
} 

void setup_network(/*uint8_t* message*/)
{	//this function should only enter once, when the node is told to setup the network initially
	//TODO: Have a case where the network is already setup
	//BLINK(LIGHT_PORT,YELLOW_LIGHT);
	/*node_status = SETTING_UP; //flag for set up routine
	neighbor_count = 0; //default it to 0 -> when routine is entered
	uint8_t target_index = 0; //default to looking at 0x0001
	//PORTD |= (1<<GREEN_LIGHT);
	send_directly_to_pi(transmit_data_buffer);
	set_downstairs_neighbor(message); //set the downstairs node for this (node that messages will be relayed to) as the person who requested this
	probe_neighbor_status(&target_index); //start by searching node 0x0001 -> must wait for a response
	wait_for_setup_response(&target_index);*/
	memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER);
	memset(recieved_data_buffer,0,PK_SZ_TXRX_BUFFER);
	send_directly_to_pi(transmit_data_buffer);
}

void continue_setup(uint8_t target) //tells the next node to do the setup routine
{
	//memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER);
	Pk_Set_Command(transmit_data_buffer,CMD_SETUP,0,0,0);
	BLINK(LIGHT_PORT,RED_LIGHT);
	send_message(target,transmit_data_buffer);
}

void set_downstairs_neighbor(uint8_t* message)
{
	downstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
	//BLINK(LIGHT_PORT,RED_LIGHT);
	++neighbor_count;
}

void set_upstairs_neighbor(uint8_t* message) //this function may prove to be my undoing. Tread carefully
{	//TODO: Neighbor count and target count must be set before this function enters. Unexpected behavior may occur otherwise
	
	//if the node has a downstairs neighbor, it needs an upstairs neighbor unless it is the last node
	//the node needs to ask the others in the field if they have a neighbor, and if not, choose that one as an upstairs neighbor
	//if a node received this request, and it has 1 neighbor, something wrong has happened
	//need to send a message and get a reply that brings us back here, incrementing the target count if
	//the node (requested) has already got neighbors
	if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 0) //this holds the number of neighbors the probed node has
	{
		node_status = IDLE;
		upstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
		++neighbor_count; //increase the amount of neighbors. Generally speaking, this should always result in 2.
		//PORTD |= (1<<YELLOW_LIGHT);
		if(neighbor_count == 2) Network_Set = true;
		continue_setup(upstairs_neighbor.id);
	}
	else 
	{	
		++target_index; //increment through the list
		if(target_index > node_count) //we have exceeded the number of nodes available without finding a free node
		{
			//this node is the last node, relay that info back to the pi
			node_status = IDLE;
			Network_Set = true;
			confirm_network_complete(message); //uses the source address	
		}
		else
		{	
			wait_for_setup_response(&target_index);
		}
	}
}

void wait_for_setup_response(uint8_t *search_index)
{
	//BLINK(LIGHT_PORT,RED_LIGHT);
	uint8_t overflow_counter = 0; //for timer
	uint8_t repeat_counter = 0; //this is due to the fact that nodes upon wakeup tend to ignore their first message (?)
	//timers
	PRR |= (0<<PRTWI); //write 1 to the PRTWI bit on PRR (power reduction register) to ensure clock is running: Should default to 0. This is just in case?
	TCCR0A |= (1<<CTC0); //set CTC bit (clear timer compare)
	TCCR0A |= (1<<CS02) | (1<< CS00); //clock select 1024 prescaler
	//TIMSK0 |= (1<<TOIE0);
	TCNT0 = 0; //set the 8 bit timer to 0
	while(node_status == SETTING_UP) //flag was set from setup network -> TODO: This may be an infinite loop. Tread carefully.
	{	
		mrf_check_flags(&handle_rx,&handle_tx); //listen for a response
		while((TIFR0 & 0x01) == 0); //while the overflow hasn't occurred don't do anything. -> could check flags here?
		TCNT0 = 0; //write 0 to counter
		TIFR0 = 0x01; //clear overflow flag by writing 1 (per the data sheet)
		++overflow_counter;
		if (overflow_counter >= 15) //from calculator: 39 = ~10 second wait
		{
			BLINK(LIGHT_PORT,RED_LIGHT);
			++repeat_counter;
			if(repeat_counter > 2) //function has repeated twice (done because first message on reset tends to get lost for unknown reason)
			{
				++*search_index; //check the next node in the list of node ID's after searching for the same one twice
				repeat_counter = 0; //restart repeat counter
			}
			if(*search_index > node_count) //this block should be outside of the timer loop, I think?
			{
				//BLINK(LIGHT_PORT,GREEN_LIGHT); BLINK(LIGHT_PORT, RED_LIGHT); BLINK(LIGHT_PORT, GREEN_LIGHT);
				node_status = IDLE;
				Network_Set = true;
				//PORTD &= ~(1<<GREEN_LIGHT);
				send_downstream(transmit_data_buffer);
				//relay the network complete info to the pi
			}
			else//case where the repeat counter is below 1 (hasn't repeated) and there are still nodes to search
			{	//is this loop entering?
				BLINK(LIGHT_PORT,GREEN_LIGHT);
				if(node_list[*search_index] == 0x0002) BLINK(LIGHT_PORT,YELLOW_LIGHT); //testing
				probe_neighbor_status(search_index); //send a message
				overflow_counter = 0; //restart overflow counter
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
	memset(buff,0,PK_SZ_TXRX_BUFFER);
	//Pk_Set_Command(buff,CMD_TO_PI,1,2,3);
	send_message(pi_address,buff);
	mrf_reset(); //experimenting
	//mrf_init(); //do I need this?	
}
void send_downstream(uint8_t* buff)
{
	if(Network_Set)
	{
		Pk_Set_Command(buff,CMD_TO_PI,0,0,0);
		send_message(downstairs_neighbor.id,buff);
	}

}

void confirm_network_complete(uint8_t* message)
{
	Pk_Set_Command(transmit_data_buffer,CMD_NETWORK_COMPLETE,0,0,0);
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),transmit_data_buffer);	
}
void probe_neighbor_status(uint8_t *target_index) 
{
	BLINK(LIGHT_PORT,YELLOW_LIGHT);
	clear_buffer(transmit_data_buffer);
	if(node_list[*target_index] == THIS_DEVICE) ++target_index; //to avoid possibly sending a message to itself
	Pk_Set_Command(transmit_data_buffer,CMD_PROBE_NEIGHBORS,0,0,0);
	send_directly_to_pi(transmit_data_buffer); //for troubleshooting
	send_message(node_list[*target_index],transmit_data_buffer);
}
void confirm_neighbor(uint8_t* message) //responds to a request for the number of neighbors
{
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	Pk_Set_Command(transmit_data_buffer,CMD_NEIGHBOR_COUNT,neighbor_count,0,0);
	//transmit_data_buffer[PK_COMMAND_HEADER+ PK_CMD_DATA_0] = neighbor_count;
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]), transmit_data_buffer); //return this to the requester
	//clear_buffer(transmit_data_buffer); //may remove later
}

void send_message(uint16_t target, uint8_t* buff)
{
	memset(buff,0,PK_SZ_TXRX_BUFFER);
	//BLINK(LIGHT_PORT,YELLOW_LIGHT);
	//Pk_Set_Packet_Size(buff,PK_SZ_TXRX_BUFFER); //this should be a function that finds the packet size maybe?
	//Pk_Set_Target_Node(buff,target);
	//Pk_Set_Src_Node(buff,THIS_DEVICE);
	uint8_t temp_buff[80];
	memset(temp_buff,1,80);
	mrf_send16(target,temp_buff,80);
	memset(buff,0,PK_SZ_TXRX_BUFFER);
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
	
	BLINK(LIGHT_PORT,GREEN_LIGHT);
}


int main(void)
{
    setup();
    while (node_status == IDLE) 
    {
		mrf_check_flags(&handle_rx, &handle_tx); //check for interrupts
    }
}

