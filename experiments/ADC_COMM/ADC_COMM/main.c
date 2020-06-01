/*
 * ADC_COMM.c
 *
 * Created: 10/29/2019 5:41:28 PM
 * Author : Michael
 * This will serve as the code to send a message with an ADC value to a node which can only send/recieve
 * On press: ADC is sampled and stored into a message
 * Message is sent
 * If send is successful, light will indicate
 *
 */ 

#include <avr/io.h>
#include "spi-master-adc1.h"
#include "tinyspi.h"
#include "mrf24j.h"
#include "blinkin.h"
#include <util/delay.h>

#define BUTTON_PIN PIND
#define BUTTON PD0

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

#define THIS_DEVICE 0x0026 //change this on each node you program

uint16_t message; //this is the message that will be sent

void setup(){
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1 << GREEN_LIGHT);
	DDRD |= (1 << BLUE_LIGHT);
	DDRD &= ~(1<<BUTTON); //set PD0 as input for the button
	
	PORTB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS); //set these ports to high (required)
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS);  //set these to output
	DDRB &= ~(1<<SPI_MISO);	//master in slave out, input on attiny
	
	//PORTD |= (1<<GREEN_LIGHT); //for some reason this turns on all the LEDs
	//PORTD &= ~(1<<GREEN_LIGHT);
	//PORTD &= ~(1<<BLUE_LIGHT);
	//PORTD &= ~(1<<YELLOW_LIGHT);
	//PORTD &= ~(1<<RED_LIGHT);
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	spi_setup();
	BLINK(LIGHT_PORT,GREEN_LIGHT);
	BLINK(LIGHT_PORT,BLUE_LIGHT);
	mrf_reset();
	BLINK(LIGHT_PORT,RED_LIGHT);
	mrf_init();
	BLINK(LIGHT_PORT,RED_LIGHT);
	
	mrf_set_pan(ASMP_PANID);
	mrf_address16_write(THIS_DEVICE); //do I need this??
	sei(); //starts interrupts, essential to let chip know message is being handled
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
}

uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON)) return 0;
	else return 1;
}

void handle_rx(){
	if(mrf_get_bufferPHY()){} //not needed
	
	uint8_t * rx_data = mrf_get_rxdata();
	
	//look at the rx_data and make a decision
}

void handle_tx(){ //a successful transmission involves sending and receiving a message.
	//This loop never seems to happen??
	if(mrf_tx_ok()){
		BLINK(LIGHT_PORT,GREEN_LIGHT);
		BLINK(LIGHT_PORT,GREEN_LIGHT);
	}
	else{
		//transmitted and not received acknowledgment
		BLINK(LIGHT_PORT,RED_LIGHT);
		BLINK(LIGHT_PORT,BLUE_LIGHT);
		BLINK(LIGHT_PORT,RED_LIGHT);
		
	}
}


void loop()
{	
	if(!pollButton())
	{	
		BLINK(LIGHT_PORT,GREEN_LIGHT);
		message = get_adc_value(0); //might need to typecast message to 8 bit pointer
		mrf_check_flags(&handle_rx, &handle_tx);
		mrf_send16(DEVICE_32,(uint8_t*)&message,2); //send the message (ADC value from channel 0) to device 32.
		BLINK(LIGHT_PORT,GREEN_LIGHT);
		mrf_check_flags(&handle_rx, &handle_tx);
	}
	mrf_check_flags(&handle_rx, &handle_tx);
}

int main(void)
{
	setup();
	//PORTD |= (1<<GREEN_LIGHT);
    /* Replace with your application code */
    while (1) 
    {
		loop();
    }
}

