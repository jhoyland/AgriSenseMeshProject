/*
 * NodeComm-5-11.c
 *
 * Created: 5/11/2020 2:43:48 PM
 * Author : Michael
 * Ensuring the nodes are wired correctly for communication
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "bitmanip.h"
#include "blinkin.h"
#include "command_specs_2_20_2020.h"
#include "mrf24j.h"
#include "packet_specs.h"
#include "PinDefs_2_18_2020.h"
#include "tinyspi.h"

#define F_CPU 1000000UL  // 1 MHz
#define ASMP_PANID 0xCAFE //sets ID for entire network
#define DEVICE_01 0x0001
#define DEVICE_02 0x0002

#define THIS_DEVICE DEVICE_01

uint16_t message; //this is the message that will be sent
uint8_t target_address;
uint8_t running_status = 0;
uint8_t* transmit_command_header;

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER]; //buffers
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER];


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
	
	transmit_command_header = & transmit_data_buffer[PK_COMMAND_HEADER]; //from sensenode.c
	
	
	//neighbor_status = STATUS_NO_NEIGHBORS;  //default to no neighbors on boot
	//command_status = STATUS_STANDBY;		//default to standby on successful startup
	//BLINK(LIGHT_PORT,YELLOW_LIGHT);
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	//BLINK(LIGHT_PORT,RED_LIGHT);
	
}

ISR(INT0_vect) //for when the MRF interrupts (sending or receiving a message)
{
	running_status |= (1<<RU_INTERRUPT);
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	mrf_interrupt_handler();
	//BLINK(LIGHT_PORT,RED_LIGHT);
	running_status &= ~(1<<RU_INTERRUPT);
}

void handle_rx()
{
	//BLINK(LIGHT_PORT,YELLOW_LIGHT); //just blink if you get the message
	running_status |= (1<<RU_RX_HANDLE);

	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()*sizeof(uint8_t)); //makes a copy of the rx data to a buffer
	if( bytes_to_word(&recieved_data_buffer[PK_DEST_ADDR_HI]) == THIS_DEVICE)
	{
		BLINK(LIGHT_PORT,RED_LIGHT);
	}
	running_status &=~(1<<RU_RX_HANDLE);
}

void handle_tx()
{
	//nothing now
}
int main(void)
{
    /* Replace with your application code */
	setup();
    while (1) 
    {
		//mrf_send16(DEVICE_02,transmit_data_buffer,2);
		mrf_check_flags(&handle_rx,&handle_tx);
		//_delay_ms(100);
		//BLINK(LIGHT_PORT,YELLOW_LIGHT);
    }
}

