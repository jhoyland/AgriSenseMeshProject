/*
 * ADC_RECIEVE.c
 *
 * Created: 10/29/2019 7:37:08 PM
 * Author : Michael
 * This is the code for a node that will receive the ADC value as a message
 * Key things to look for with this program is determining if the message comes out accurately
 * and controlling the device based on the message received
 * What will happen:
 * A blue light will come on, letting the user know the node is looking for data
 * when the message is recieved, the blue light will go off
 * the green or the LED will come on depending on the value of the message sent
 */ 

#include <avr/io.h>
#include "PinDefs_2_18_2020.h"
#include "tinyspi.h"
#include "blinkin.h"
#include "mrf24j.h"

#define F_CPU 1000000UL
#define LIGHT_PORT PORTD
#define BLUE_LIGHT PD4
#define YELLOW_LIGHT PD6
#define RED_LIGHT PD7
#define GREEN_LIGHT PD5

#define CS_SR	PB3 /*Shift Register*/ //MOSI line
#define ADC_CS	PB7 /* MCP3208 */

//required for ADC to work??
#define DO	PB1
//#define USCK PB2 //get rid of??
#define D1	PB0

#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK PB5
#define SPI_SS PB2


#define ASMP_PANID 0xcafe //sets ID for entire network
#define DEVICE_26 0x0026
#define DEVICE_32 0x0032
#define DEVICE_29 0x0029

#define THIS_DEVICE 0x0029 //change this on each node you program

uint16_t message; //this is the message that will be recieved


void setup()
{
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1 << GREEN_LIGHT);
	DDRD |= (1 << YELLOW_LIGHT);
	//DDRD |= (1 << BLUE_LIGHT);
	
	PORTB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS); //set these ports to high (required)
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS);  //set these to output
	DDRB &= ~(1<<SPI_MISO);	//master in slave out, input on attiny
	
	//BLINK(LIGHT_PORT,GREEN_LIGHT);
	spi_setup();
	//BLINK(LIGHT_PORT,GREEN_LIGHT);

	mrf_reset();
	BLINK(LIGHT_PORT,YELLOW_LIGHT);
	mrf_init(); //this function cannot complete??
	BLINK(LIGHT_PORT,YELLOW_LIGHT);

	
	mrf_set_pan(ASMP_PANID);
	mrf_address16_write(THIS_DEVICE); //do I need this??
	
	sei(); //starts interrupts, essential to let chip know message is being handled
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
	BLINK(LIGHT_PORT,YELLOW_LIGHT);
}
ISR(INT0_vect)
{
	mrf_interrupt_handler();
}

void handle_rx(){
	//if(mrf_get_bufferPHY()){} //not needed
	//PORTD |= (1<<BLUE_LIGHT); //this does not light up; this loop never enters
	uint8_t * rx_data = mrf_get_rxdata();
	if(*rx_data >= 100) BLINK(LIGHT_PORT,GREEN_LIGHT);
	
	if(*rx_data < 100) BLINK(LIGHT_PORT, RED_LIGHT);
	
	
	//look at the rx_data and make a decision
}

void handle_tx(){ //a successful transmission involves sending and receiving a message.
	if(mrf_tx_ok()){
	}
	else{
		//transmitted and not received acknowledgment
	}
}

int main(void)
{
	setup();
    /* Replace with your application code */
	//BLINK(LIGHT_PORT,BLUE_LIGHT);
	while (1) 
    {	
		mrf_check_flags(&handle_rx, &handle_tx); //is this happening too quickly??
    }
}

