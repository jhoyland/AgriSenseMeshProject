/*
 * ADC_Tester.c
 *
 * Created: 10/1/2019 7:00:40 PM
 * Author : Michael
 * PURPOSE: Press a button, read an ADC value, and when the value passes at threshold, activate a light
 */ 
#define F_CPU 8000000UL

#include <avr/io.h>
#include "tinyspi.h"
#include "spi-master-adc1.h"
#include <util/delay.h>

#define LIGHT_PORT PORTD
#define BUTTON PD0
#define BUTTON_PIN PIND
#define BLUE_LIGHT PD4
#define RED_LIGHT PD7
#define GREEN_LIGHT PD5

#define YELLOW_LIGHT PD6
/*
#define RED_LIGHT PD5
#define YELLOW_LIGHT PD6
#define GREEN_LIGHT PD7*/

#define CS_SR	PB3 /*Shift Register*/
#define ADC_CS	PB7 /* MCP3208 */
#define DO	PB1
//#define USCK PB2 //get rid of??
#define D1	PB0

#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK PB5
#define SPI_SS PB2



void setup(){
	DDRD |= (1 << RED_LIGHT); //set PD7 to output for LED
	DDRD |= (1<< GREEN_LIGHT);
	DDRD |= (1 << BLUE_LIGHT);
	DDRD &= ~(1<<BUTTON); //set PD0 as input for the button
	
	PORTB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS);
	DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<ADC_CS) | (1<<SPI_SS);
	DDRB &= ~(1<<SPI_MISO);	//master in slave out, input on attiny
	
	
	spi_setup();
}

uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON)) return 0;
	else return 1;
}

void loop(){
	
//if(!(pollButton())) {PORTD |= (1<<LIGHT); _delay_ms(100);}
	//else PORTD &= ~(1<<LIGHT);
	if(!pollButton()){
		if(get_adc_value(0) > 100)
		{
		PORTD |= (1<<GREEN_LIGHT); //if the ADC value returns, light green
		PORTD &= ~(1<<RED_LIGHT);
		//PORTD |= (1<<BLUE_LIGHT);
		//_delay_ms(1000);
	    }
		else 
		{
			PORTD &= ~(1<<GREEN_LIGHT);
			PORTD |= (1<<RED_LIGHT); //if ADC value doesn't return, light red
			//PORTD |= (1<<BLUE_LIGHT);
		}
	
	/*LIGHT BLINKING DEBUG*/
	/*PORTD |= (1<<LIGHT);
	_delay_ms(1000);
	PORTD &= ~(1<<LIGHT);
	
	PORTD|= (1<<GREEN_LIGHT);
	_delay_ms(1000);
	PORTD &= ~(1<<GREEN_LIGHT);*/
	}
}
int main(void)
{

	setup();
	
    /* Replace with your application code */
    while (1) 
    {
		loop();
		
	}
	
}



