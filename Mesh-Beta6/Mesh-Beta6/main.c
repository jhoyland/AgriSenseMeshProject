/*
 * Mesh-Beta6.c
 *
 * Created: 6/17/2020 12:51:09 PM
 * Author : Michael
 * Attempting to include network setup and other functionalities to the relay system
 */ 

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

uint8_t node_count = 5; //holds the number of nodes that are in the network (eventually read/write from file)
uint8_t neighbor_count; //holds the number of neighbors a node has (should max be 2)
uint8_t target_count; //holds the number of nodes searched in a network 
uint8_t node_status; //holds the current node status
uint8_t pi_address = 99; //should I make this 3142 or something?
bool Network_Set;

uint8_t running_status = 0;
struct neighbor downstairs_neighbor;
struct neighbor upstairs_neighbor;

struct neighbor
{
	uint8_t id;
	uint8_t sensors; //a bitmask for the sensors available??
	uint8_t upstairs_id; //this node's "upstream" neighbor
	uint8_t downstairs_id; //this node's "downstream" neighbor
};


void COMMAND_HANDLER(uint8_t* message) //takes the entire message and decides what to do with it
{
	switch(bytes_to_word(&message[PK_COMMAND_HEADER+PK_CMD_HI])) //look at the command given to the node
	{
		case CMD_PING: //node is being pinged
		ping_handler(message);
		break;
		case CMD_ECHO: //node responded to a ping
		echo_handler(message);
		break;
		case CMD_DATA:
		//collect_data(message); //collect data from the required sensors
		break;
		case CMD_SETUP:
		setup_network(message);//node is now in network setup mode
		break;
		case CMD_PROBE_NEIGHBORS: //node has received a request to return the number (and ID?) of its neighbors
		confirm_neighbor(message);
		break;
		case CMD_NEIGHBOR_COUNT: //has received a response containing the number of neighbors
		set_upstairs_neighbor(message);
		break;
		default:
		//command not recognized
		break;	
	}
	
}


void ping_handler(uint8_t* message)
{ 
	//this is to determine if the chain is still present
	if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 0)
	{ //first "Accessory bit" is 0: This is just a boring ping									  
		Pk_Set_Command(message,CMD_ECHO,0,0,0); //respond with "echo" in the command location
		send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),message); //send it back to the originator
	}
	if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 1)
	{ //first accessory bit is 1: This is a ping for network setup
	  //need to return the number of neighbors. Later idea: also include their ID's?
	}
}

void echo_handler(uint8_t* message)
{
	
} 

void setup_network(uint8_t* message)
{ //network setup protocol: Depending on if this function was entered to add a lower node or an upper node
	neighbor_count = 0; //this function should only enter once, when the node is told to setup the network
	target_count = 1;
	set_downstairs_neighbor(message);
	probe_neighbor_status(target_count); //start by searching node 1
	//set_upstairs_neighbor(message,target_count);
}

void continue_setup(uint8_t target) //tells the next node to do the setup routine
{
	memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER);
	Pk_Set_Command(transmit_data_buffer,CMD_SETUP,0,0,0);
	send_message(target,transmit_data_buffer);
}

void set_downstairs_neighbor(uint8_t* message)
{
	downstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
	DDRD |= (1<<RED_LIGHT);
	++neighbor_count;
}

void set_upstairs_neighbor(uint8_t* message) //this function may prove to be my undoing. Tread carefully
{	//TODO: Neighbor count and target count must be set before this function enters. Unexpected behavior may occur otherwise
	//if the node has a downstairs neighbor, it needs an upstairs neighbor unless it is the last node
	//the node needs to ask the others in the field if they have a neighbor, and if not, choose that one as an upstairs neighbor
	//need to send a message and get a reply that brings us back here, incrementing the target count if
	//the node (requested) has already got neighbors
	if(message[PK_COMMAND_HEADER+PK_CMD_DATA_0] == 0)
	{
		upstairs_neighbor.id = bytes_to_word(&message[PK_SRC_ADDR_HI]);
		++neighbor_count;
		DDRD |= (1<<YELLOW_LIGHT);
		continue_setup(upstairs_neighbor.id);
	}
	else 
	{	
		++target_count; //increments through the list
		if(target_count > node_count) //we have exceeded the number of nodes available without finding a free node
		{
			//this node is the last node, relay that info back to the pi
			confirm_network_complete(message); //uses the source address	
		}
		else
		{
			probe_neighbor_status(target_count);	
		}
	}
}

/*void collect_data(uint8_t* message)
{
	//depending on the sensors requested, add data	
}*/

void confirm_network_complete(uint8_t* message)
{
	Pk_Set_Command(transmit_data_buffer,CMD_NETWORK_COMPLETE,target_count,0,0);
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),transmit_data_buffer);	
}
void probe_neighbor_status(uint16_t target) 
{
	Pk_Set_Command(transmit_data_buffer,CMD_PROBE_NEIGHBORS,0,0,0);
	send_message(target,transmit_data_buffer);
}
void confirm_neighbor(uint8_t* message) //responds to a request for the number of neighbors
{
	Pk_Set_Command(transmit_data_buffer,CMD_NEIGHBOR_COUNT,0,0,0);
	transmit_data_buffer[PK_COMMAND_HEADER+ PK_CMD_DATA_0] = neighbor_count;
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]), transmit_data_buffer);
	clear_buffer(transmit_data_buffer); //may remove later
}

void send_message(uint16_t target, uint8_t* buff)
{
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

void handle_rx()
{
	running_status |= (1<<RU_RX_HANDLE);
	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()); //makes a copy of the rx data to a buffer
	//check the addressing bit to determine what should be done
	if(bytes_to_word(&recieved_data_buffer[PK_DEST_ADDR_HI]) == THIS_DEVICE) //a message specifically for this node
	{
		BLINK(LIGHT_PORT,GREEN_LIGHT);
		COMMAND_HANDLER(recieved_data_buffer); //puts the received data buffer into this
	}
	else
	{
	//this message isn't for this node, do nothing
	}

	running_status &= ~(1<<RU_RX_HANDLE);
}

void handle_tx()
{
	//doesn't really need to do anything upon sending a packet... yet...?
}

void setup()
{
	
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1 << YELLOW_LIGHT);
	DDRD |= (1 << GREEN_LIGHT);
	
	PORTB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS) | (1<<MRF_CS) ; //set these ports to high (required)
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS) | (1<<MRF_CS) ;  //set these to output
	DDRB &= ~(1<<SPI_MISO);	//master in slave out, input on attiny
	
	spi_setup();
	mrf_reset();
	mrf_init();
	
	mrf_set_pan(ASMP_PANID); //set PANID
	mrf_address16_write(THIS_DEVICE); //set device address
	sei(); //starts interrupts, essential to let chip know message is being handled
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
	
	memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset, possibly not needed
	memset(recieved_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset
	
	//Network_Set = FALSE; //default to network setup: No down or up neighbor
	target_count = 0; //setup resets these values to 0
	neighbor_count = 0; //setup resets these values to 0
	
	BLINK(LIGHT_PORT,GREEN_LIGHT);
}


int main(void)
{
    setup();
    while (1) 
    {
    }
}

