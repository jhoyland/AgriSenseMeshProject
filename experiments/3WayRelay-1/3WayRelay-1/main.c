/*
 * 3WayRelay-1.c
 *
 * Created: 2/20/2020 11:21:40 AM
 * Author : Michael
 * This is a prototype of the network setup algorithms
 * This works for 3 nodes
 */ 

#include <avr/io.h>
#include <util/delay.h>
//TODO: Check if the PinDefs file is in line with the board
#include "PinDefs_2_18_2020.h"
#include "command_specs_2_20_2020.h"
#include "tinyspi.h"
#include "mrf24j.h"
#include "bitmanip.h"
#include "node_statuses.h"
#include "packet_specs.h"
#include <string.h>
#include "blinkin.h"
#include "Packet_Setup.h"

#define ASMP_PANID 0xCAFE //sets ID for entire network
#define DEVICE_01 0x0001
#define DEVICE_02 0x0002
#define DEVICE_03 0x0003
#define MAX_DEVICE_NUMBERS 3

#define THIS_DEVICE 0x0001 //change this on each node you program
#define LAST_NODE DEVICE_03

//uint16_t message; //this is the message that will be sent
uint8_t target_address;

uint8_t* active_command;
uint8_t* transmit_command_header;

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER]; //buffers
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER];

uint8_t neighbor_status; //flag for holding if a node has the right number of neighbors
uint8_t command_status;  //flag for reading what the command status is
uint8_t node_status;
uint8_t running_status = 0;
uint8_t UPPER_NEIGHBOR_ADDRESS = 0x0000; //default to no neighbors
uint8_t LOWER_NEIGHBOR_ADDRESS = 0x0000; //default to no neighbors



void setup()
{
	
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1 << YELLOW_LIGHT);
	DDRD |= (1 << GREEN_LIGHT);
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	
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
	
	transmit_command_header = &transmit_data_buffer[PK_COMMAND_HEADER]; //from sensenode.c
	
	neighbor_status = STATUS_NO_NEIGHBORS;  //default to no neighbors on boot
	command_status = STATUS_STANDBY;		//default to standby on successful startup
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	
}



void handle_tx()
{
	
}


ISR(INT0_vect) //for when the MRF interrupts (sending or receiving a message)
{
	running_status |= (1<<RU_INTERRUPT);
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	mrf_interrupt_handler();
	running_status &= ~(1<<RU_INTERRUPT);
}


	
//PINGS:
//REQUESTING A RESPONSE: tell it to "echo"
//RESPONDING: say "ping"
/*void ping(uint16_t target, uint8_t* buff)
{	//node_status = 1; //currently waiting for something
	word_to_bytes(buff[PK_CMD_HI],CMD_ECHO);//puts the echo command into the command header
	mrf_send16(target,buff,buff[PK_COMMAND_HEADER] + message[PK_SZ_PACKET]); //message saying ECHO
}*/


void ping_respond(uint16_t target, uint8_t* buff)
{
	//this command should just send back to the neighbor that it was pinged.
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	//put info into the sending buffer
	Set_Packet_Size(buff,PK_SZ_ADDR_HEADER+PK_SZ_CMD_HEADER);
	Set_Target_Node(buff,target);
	Set_Command(buff,CMD_PING,1,2,3,4); //1,2,3,4 are just random numbers for this
	
	//testing to see if the sending package has nothing in it...
	if(bytes_to_word(&buff[PK_COMMAND_HEADER+PK_CMD_HI]) == CMD_PING) BLINK(LIGHT_PORT,YELLOW_LIGHT);
	
	mrf_send16(target,buff,buff[PK_COMMAND_HEADER + PK_SZ_PACKET]);
	
}


void COMMAND_HANDLER(uint8_t* message) //the received data buffer goes in here
{
	//BLINK(LIGHT_PORT,RED_LIGHT);
	//this only enters from handle_rx, confirms the message is for this device. A message has been recieved.
	//CASES:
	/*-------------------------------GENERIC PING----------------------------------------------------------------*/
	//PING: Should respond that a ping was received by the correct node.
	if( (bytes_to_word(&message[PK_COMMAND_HEADER + PK_CMD_HI]) == CMD_PING) )// && (bytes_to_word(&message[PK_COMMAND_HEADER + PK_CMD_DATA_1]) == 0x0000)) //no extra command data, just echo
	{
		BLINK(LIGHT_PORT,YELLOW_LIGHT);
		//respond to the PI with your own message
		ping_respond(bytes_to_word(&message[PK_SRC_ADDR_HI]),transmit_data_buffer); //respond with the transmit data buffer
		_delay_ms(100);
		//respond to another node
		if(THIS_DEVICE != DEVICE_02) ping_respond(DEVICE_02,transmit_data_buffer); //testing to see if I'm sending garbage
	}
	//if a node receives "PING" (should only get it after asking for an echo)
	//if( (bytes_to_word(&message[PK_CMD_HI]) == CMD_PING) && (bytes_to_word(&message[PK_CMD_DATA_0]) == 0x0000) )
	//{
		//no extra command data
		//just received "ping"
		//need to let the node sending it know that 2-way communication is established. This means a downstairs neighbor exists
		//and the lower node should have an upstairs neighbor
		//BLINK(LIGHT_PORT,GREEN_LIGHT);
		//word_to_bytes(&transmit_command_header[PK_CMD_DATA_0],CMD_ACK);
	//}
	/*----------------------------------END GENERIC PING---------------------------------------------------------*/
	
	
	
	/*---------------------------SET NEIGHBORS------------------------------------------------------------*/
	//Command is "Set neighbors"
	//
	//if on standby and has no neighbors, need to set the source address as the lower neighbor
	//if on standby and has one neighbor, needs to search for a higher neighbor
	if( (bytes_to_word(&message[PK_CMD_HI]) == CMD_SET_NEIGHBOR) && (neighbor_status == STATUS_NO_NEIGHBORS) && (THIS_DEVICE != LAST_NODE)) //asking for a neighbor, node has no neighbors
	{
		command_status =  CMD_SET_NEIGHBOR;
		LOWER_NEIGHBOR_ADDRESS = bytes_to_word(&message[PK_SRC_ADDR_HI]); //set the lower neighbor address to the source address
		neighbor_status = STATUS_ONE_NEIGHBOR; // now has a downstairs neighbor.
		//now needs to find an upstairs neighbor
		
	}
	
	//not the last node, was told to set neighbor, and has a high neighbor. Time to look for an upstream neighbor.
	//must ping a neighbor, and if it responds, set it as an upstream neighbor
	if ( (bytes_to_word(&message[PK_CMD_HI]) == CMD_SET_NEIGHBOR) && (neighbor_status == STATUS_ONE_NEIGHBOR) && (THIS_DEVICE != LAST_NODE) )
	{
		
		//for(int i = THIS_DEVICE+1; i < MAX_DEVICE_NUMBERS; i++)
		//{
		//	word_to_bytes(message[PK_CMD_HI],CMD_ECHO);
		//	word_to_bytes(message[PK_CMD_DATA_0],CMD_SET_NEIGHBOR); //include that we're searching for neighbors
		//}
	}
	if ((bytes_to_word(&message[PK_CMD_HI]) == CMD_SET_NEIGHBOR) && (neighbor_status == STATUS_TWO_NEIGHBORS))
	{
		//shouldn't do anything
	}
	/*--------------------------------END SET NEIGHBORS--------------------------------------------*/
	
	
	
	/*---------------------------------COLLECT DATA-------------------------------------------------*/
	if(bytes_to_word(&message[PK_CMD_HI]) == CMD_DATA && (THIS_DEVICE == LAST_NODE))
	{
		command_status = STATUS_ACQUISITION_READ;
	}
	/*--------------------------------END COLLECT DATA---------------------------------------------*/
}

void handle_rx()
{
	BLINK(LIGHT_PORT,RED_LIGHT);
	running_status |= (1<<RU_RX_HANDLE);

	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()*sizeof(uint8_t)); //makes a copy of the rx data to a buffer

	//check the addressing bit to determine what should be done
	if(bytes_to_word(&recieved_data_buffer[PK_DEST_ADDR_HI]) == THIS_DEVICE ) //a message specifically for this node
	{
		BLINK(LIGHT_PORT,GREEN_LIGHT);
		//enqueue( &command_queue, &recieved_data_buffer[PK_COMMAND_HEADER] ); 
		// TO DO: Check that enqueue worked. Currently the command is lost if queue is full - should send error to base in this case
		COMMAND_HANDLER(recieved_data_buffer); //puts the received data buffer into this
	}
	else
	{
		if( bytes_to_word(&recieved_data_buffer[PK_DEST_ADDR_HI]) == CMD_SET_NEIGHBOR); //we are in network setup mode. This message comes
		//from a raspberry pi, or another node, to all nodes
		//should probably give a unique identifier for this??
		//word_to_bytes(&transmit_data_buffer[PK_CMD_HI], )
		//send_error(bytes_to_word(& recieved_data_buffer[PK_DEST_ADDR_HI]));
		/*
		recieved_data_buffer[PK_COMMAND_HEADER + PK_HOP_COUNT] ++;
		if(recieved_data_buffer[PK_DEST_ADDR_HI] == PI_ADDR_HI && recieved_data_buffer[PK_DEST_ADDR_LO] == PI_ADDR_LO)
			send_downstream(recieved_data_buffer);
		else send_upstream(recieved_data_buffer);*/

	}

	running_status &= ~(1<<RU_RX_HANDLE);
}


void loop()
{
	
}
int main(void)
{
	setup();
    while (1) 
    {
		mrf_check_flags(&handle_rx,&handle_tx);
		//_delay_ms(10);
		//BLINK(LIGHT_PORT,RED_LIGHT);
    }
}

