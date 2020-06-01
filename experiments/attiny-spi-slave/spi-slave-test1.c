
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

volatile char cmdcode = 0;

void setup_spi(void)
{
	cli(); /*Disable interrupts*/

	DDRB |= 1<<DO;  

	USICR = ((1<<USIWM0)|(1<<USICS1)); /* 3-wire mode and external idle-low clock  */
	
	PORTB |= 1<<CS; /* Pull up resistor enabled on CS */

	PCMSK |= 1<<CS; /* Interrupt will be set on pin changes of CS */
	GIMSK |= 1<<PCIE; /* Pin change interrupts will be enabled */

	_delay_ms(500);

	sei(); /* Enable interrupts */
}

ISR(PCINT0_vect)
{
	if( PINB & (1<<CS) ) /*If true then CS pin has just gone high - SPI transaction finished */
	{
		USICR &= ~(1<<USIOIE); /* disble the overflow interrupt for the 4-bit counter */
	}
	else   /* Start of an SPI transaction */
	{
		cmdcode = 0; /* prepares for receiving initial byte */
		USIDR = 0;
		USICR |= (1<<USIOIE);
		USISR = 1<<USIOIF;   /* resets the overflow flag - weirdly  */
	}

}

ISR(USI_OVF_vect)
{
	switch(cmdcode){
		case 0:  /* first time call at the end of the first byte */
			cmdcode = USIDR; /* read the command */
			USISR = 1<<USIOIF; /*clear the overflow */
			break;
		case 'A': /* end of second byte - probably want to change this! seems like there is a wasted byte here  */

			USIDR = 5;
			USISR = 1<<USIOIF;
			break;

		case 'B':

			USIDR = 9;
			USISR = 1<<USIOIF;
			break;

		default:

			USIDR = cmdcode;
			USISR = 1<<USIOIF; 

	}
}


int main()
{	
	setup_spi();
	for(;;);
	return 0;

}
