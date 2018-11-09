/*
 * ButtonComm.c
 *
 * Created: 10/19/2018 6:05:44 PM
 * Author : Michael
 * This file will respond to a button push
 * it will send a message, and light an LED to indicate the message has been sent
 * if it receives a message, it will blink another LED to indicate the message has been recieved
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "tinyspi.h"
#include "mrf24j.h"
#include "mrfpindefs.h"
#include "blinkin.h"

#define LED_PORT PORTD
#define SEND_LED PD3
#define RECEIVE_LED PD4
#define RED_LED PD7

#define BUTTON PB0
#define BUTTON_PIN PINB

#define ASMP_PANID 0xcafe //sets ID for entire network

char* msg = "aaaa";


//source and destination addresses for the devices built so far
#define DEVICE_31 0x0031
#define DEVICE_16 0x0016
#define DEVICE_13 0x0013

#define THIS_DEVICE = DEVICE_31 //CHANGE THIS EACH TIME YOU PROGRAM


void handle_rx(){
	if(mrf_get_bufferPHY()){} //not needed
		
	uint8_t * rx_data = mrf_get_rxdata();
	
	if(rx_data[0] == 'a'){
		BLINK(LED_PORT, RECEIVE_LED);
	}
	else{
		//blink LED for different message
	}
}

void handle_tx(){ //a successful transmission involves sending and receiving a message.
	if(mrf_tx_ok()){
		BLINK(LED_PORT, SEND_LED);
		BLINK(LED_PORT, RECEIVE_LED);
		BLINK(LED_PORT, SEND_LED);
	}
	else{
		//transmitted and not received acknowledgment
		BLINK(LED_PORT,RECEIVE_LED);
		BLINK(LED_PORT,RECEIVE_LED);
	}
}

void setup(){
	// data directions
	
	DDRB |= (1<<MRF_WAKE) | (1<<MRF_RESET) | (1<<MRF_CS);
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK);
	DDRB &= ~(1<<BUTTON);

	DDRD |= (1<<SEND_LED) | (1<<RECEIVE_LED) | (1<<BUTTON_LED); //sets LED pins to outputs
	
	PORTB |= (1<<BUTTON) | (1<<MRF_CS) | (1<<MRF_INT); //makes button pin high
	MRF_RESET_PORT |= (1<<MRF_RESET);
	spi_set_data_direction(SPI_MSB);
	spi_setup();
	
	mrf_reset();
	mrf_init();
	
	mrf_set_pan(ASMP_PANID);
	mrf_address16_write(SRC_ADDRESS);
	sei(); //starts interrupts, essential to let chip know message is being handled
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
	
}

uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON)) return 0;
	else return 1;
}


ISR(INT0_vect){
	mrf_interrupt_handler(); //mrf24 object interrupt routine
}

void loop(){
	mrf_check_flags(&handle_rx, &handle_tx);
	if(pollButton()){
		BLINK(LED_PORT,SEND_LED); //if button is pushed, LED blinks
		//on button push, send message and blink to confirm message is send
		mrf_send16(DEVICE_16, (uint8_t*)msg, 4);
		mrf_check_flags(&handle_rx, &handle_tx);	
		}	
	}
	else{
		mrf_check_flags(&handle_rx, &handle_tx);
	}
	
}

int main(void)
{
	setup();
    while (1) loop();
	return 0;
}

