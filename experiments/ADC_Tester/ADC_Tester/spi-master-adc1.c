//#define F_CPU 8000000UL
/* 8MHz causes garbled data  */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "tinyspi.h"
#include "spi-master-adc1.h"

/* 
References:

see: weathergadget.wordpress.com/2016/05/19/usi-spi-slave-communication/


*/

#define CS_SR	PB3 /*Shift Register*/
#define ADC_CS	PB7 /* MCP3208 */
#define DO	PB1
#define USCK	PB2
#define D1	PB0
#define GREEN_LIGHT PD5
#define YELLOW_LIGHT PD6
#define BLUE_LIGHT PD4


uint8_t adc_buffer[3];



uint16_t get_adc_value(uint8_t chan) //change this function to have ADC_CS as a parameter
{
	DDRD |= (1<<YELLOW_LIGHT);
	DDRD |= (1<<BLUE_LIGHT);
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	PORTD |= (1<<YELLOW_LIGHT);
	//_delay_ms(100);
	PORTD &= ~(1<<YELLOW_LIGHT);
	spi_transfer_nbytes(adc_buffer,adc_buffer,3,ADC_CS); //this function doesnt execute
	PORTD |= (1<<YELLOW_LIGHT);
	//_delay_ms(100);
	PORTD &= ~(1<<YELLOW_LIGHT);
	uint16_t b1 = adc_buffer[1] & 0xF; //magic number sets first 4 bits to 0.
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

/*void display_value(uint8_t val)
{
	uint8_t dummy;
	spi_transfer_bytes(&val,&dummy,1,CS_SR);
}*/

