/*
 * ADCTEST.c
 *
 * Created: 11/6/2018 1:32:39 PM
 * Author : Michael
 */ 
 
 //This program should just send a value from the ADC to the rasberry pi, which then prints it out on their screen.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "tinyspi.h"
#include "mrf24j.h"
#include "blinkin.h"
#include "mrfpindefs.h"
#include "bitmanip.h"


#define LED_PORT    PORTD
#define LED_1       PD3    /*REMOTE LED*/
#define LED_2       PD4    /*LOCAL LED*/
#define LED_3		PD7

#define BUTTON_1    PD5
#define BUTTON_PIN  PIND

#define MAX_DEBOUNCE_COUNT 8

#define ASMP_PANID 0xcafe

uint8_t ADC_CHANNEL = 4; //the ADC has the input in on channel 4.


uint16_t loop_counter;
uint16_t value;
uint16_t SRC_ADDRESS = 0x0023;
uint16_t DEST_ADDRESS = 0x0031;


/*Called by check flags*/
void handle_rx() {

	
	if(mrf_get_bufferPHY()){

	}

	uint8_t * rx_data = mrf_get_rxdata();

	if(bytes_to_words(rx_data[0]) == 'a')
	{
		BLINK(LED_PORT,LED_1);BLINK(LED_PORT,LED_1);
	}
	else
	{
		BLINK(LED_PORT,LED_1);
	}

}

void handle_tx() {

	
	if (mrf_tx_ok()) {
		BLINK(LED_PORT,LED_2);
		BLINK(LED_PORT,LED_1);
		BLINK(LED_PORT,LED_2);
		} else {
		BLINK(LED_PORT,LED_2);
	}
}


void setup() {

	/* Data directions */

	DDRB |= (1<<MRF_WAKE) | (1<<MRF_RESET) | (1<<MRF_CS);
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK);

	DDRD |= (1<<LED_1) | (1<<LED_2);
	
	PORTD |= (1<<BUTTON_1);
	PORTB |= (1<<MRF_CS); //
	PORTB |= (1<<MRF_INT);
	MRF_RESET_PORT |= (1<<MRF_RESET);
	spi_set_data_direction(SPI_MSB);
	spi_setup();

	mrf_reset();
	mrf_init();
	
	mrf_set_pan(ASMP_PANID);
	// This is _our_ address
	mrf_address16_write(SRC_ADDRESS);
	
	loop_counter = 0;

	// uncomment if you want to receive any packet on this channel
	//mrf_set_promiscuous(true);
	
	// uncomment if you want to enable PA/LNA external control
	//mrf_set_palna(true);
	
	// uncomment if you want to buffer all PHY Payload
	//mrf_set_bufferPHY(true);

	// attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
	// last_time = millis();
	sei();
	EIMSK |= (1<<INT0);
	EICRA |= (1<<ISC01);
	
}

ISR(INT0_vect) {
	mrf_interrupt_handler(); // mrf24 object interrupt routine
}



uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON_1)) return 0;
	else return 1;
}

void loop() {
	if(pollButton()){
		//read the ADC Value
		value = get_adc_value(ADC_CHANNEL);
		
		if(value == 0) BLINK(LED_PORT, LED_1);
		if(value >= 2000) BLINK(LED_PORT, LED_2);
	}
}



int main(void)
{
	setup();
	while(1) loop();

	return 0;
}


