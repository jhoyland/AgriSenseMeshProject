
#define F_CPU 1000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* 
References:

see: weathergadget.wordpress.com/2016/05/19/usi-spi-slave-communication/


*/

#define CS	PB3
#define DO	PB1
#define USCK	PB2
#define D1	PB0

void setup_spi(void)
{
	cli(); /*Disable interrupts*/

	DDRB |= 1<<DO;  

	USICR = ((1<<USIWM0)|(1<<USICS1)); /* 3-wire mode and external idle-low clock  */
	
	PORTB |= 1<<CS; /* Pull up resistor enabled on CS */

	PCMSK |= 1<<CS; /* Interrupt will be set on pin changes of CS */
	GIMSK |= 1<<PCIE; /* Pin change interrupts will be enabled */

	sei(); /* Enable interrupts */
}

ISR(PCINT0_vect)
{
	if( PINB & (1<<CS) ) /*If true then CS pin has just gone high - SPI transaction finished */
	{

	}
	else   /* Start of an SPI transaction */
	{
		
	}

}
