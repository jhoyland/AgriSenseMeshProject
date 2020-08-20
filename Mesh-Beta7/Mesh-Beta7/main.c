/*
 * Mesh-Beta7.c
 *
 * Created: 7/10/2020 12:00:22 PM
 * Author : Michael
 * Troubleshooting: Why are messages so big being sent?
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "PinDefs_2_18_2020.h"
#include "command_specs_2_20_2020.h"
#include "bitmanip.h"
#include "mrf24j.h"
#include "blinkin.h"
#include "Network_functions.h"
#include "Packet_Setup.h"
#include "packet_specs.h"
#include "tinyspi.h"


#define THIS_DEVICE 0x0001
#define ASMP_PANID 0xCAFE

uint16_t node_list[3] = {0x0001,0x0002,0x0003}; //list of nodes on the network
uint8_t target_index;
uint16_t pi_address = 0x3142;

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t received_data_buffer[PK_SZ_TXRX_BUFFER];

uint8_t running_status = 0;
uint8_t neighbor_count = 0;

struct neighbor
{
	uint16_t id;
	uint16_t upstairs_id;
	uint16_t downstairs_id;
	
};

struct neighbor downstairs_neighbor, upstairs_neighbor;

ISR(INT0_vect)
{
	running_status |= (1<<RU_INTERRUPT);
	mrf_interrupt_handler();
	running_status &= ~(1<<RU_INTERRUPT);
}

void handle_rx()
{
	running_status|= (1<<RU_RX_HANDLE);
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	memcpy(received_data_buffer,mrf_get_rxdata(),mrf_rx_datalength());
	COMMAND_HANDLER(received_data_buffer);
	running_status &= ~(1<<RU_RX_HANDLE);

}

void handle_tx()
{
	BLINK(LIGHT_PORT,RED_LIGHT);
}

void COMMAND_HANDLER(uint8_t* message)
{
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	//send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),transmit_data_buffer);
	uint16_t command = bytes_to_word(&message[PK_COMMAND_HEADER+PK_CMD_HI]);
	switch(command)
	{
		case CMD_SETUP:
		setup_network(message,&upstairs_neighbor,&downstairs_neighbor, &target_index, &neighbor_count, node_list);
		break;
	}

	
}

void setup_network(uint8_t* message) //keep the received message to determine the course of action
{
	send_message(bytes_to_word(&message[PK_SRC_ADDR_HI]),transmit_data_buffer); //right now just responding to a message and troubleshooting
	
}

void set_downstairs_neighbor(uint8_t* buff) //use the received buffer to set the downstairs node's ID
{
	downstairs_neighbor.id = bytes_to_word(&buff[PK_SRC_ADDR_HI]);
	++neighbor_count;
}


void send_message(uint16_t target, uint8_t* buff)
{
	Pk_Set_Src_Node(buff,THIS_DEVICE);
	Pk_Set_Target_Node(buff,target);
	Pk_Set_Packet_Size(buff,PK_SZ_TXRX_BUFFER);
	Pk_Set_Command(buff,CMD_PING,0,0,0);
	//memset(buff,1,PK_SZ_TXRX_BUFFER);
	mrf_send16(target,buff,PK_SZ_TXRX_BUFFER);
	//_delay_ms(100);
	setup(); //just a guess
}

void setup()
{
	
	DDRD |= (1 << RED_LIGHT);
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
	
	
	//transmit_command_header = &transmit_data_buffer[PK_COMMAND_HEADER]; //from sensenode.c
	//recieved_command_header = &recieved_data_buffer[PK_COMMAND_HEADER]; //allows simple pointing to the command blocks
	
	//neighbor_status = STATUS_NO_NEIGHBORS;  //default to no neighbors on boot
	//command_status = STATUS_STANDBY;		//default to standby on successful startup
	memset(transmit_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset
	memset(received_data_buffer,0,PK_SZ_TXRX_BUFFER); //clear buffer to 0 on reset
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	target_index = 0;
	
}

int main(void)
{
    /* Replace with your application code */
	setup();
    while (1) 
    {
		mrf_check_flags(&handle_rx,&handle_tx);
    }
}

