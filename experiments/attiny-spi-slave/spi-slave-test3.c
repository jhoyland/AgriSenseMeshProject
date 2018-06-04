#define F_CPU 8000000UL
/* 8MHz causes garbled data  */

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
volatile uint8_t i = 0;
volatile char byteno = 0;

void setup_spi(void)
{
        /*Disable interrupts*/

	DDRB |= 1<<DO;  

	USICR = ((1<<USIWM0)|(1<<USICS1)); /* 3-wire mode and external idle-low clock  */
	
	PORTB |= 1<<CS; /* Pull up resistor enabled on CS */

	PCMSK |= 1<<PCINT3; /* Interrupt will be set on pin changes of CS */
	GIMSK |= 1<<PCIE; /* Pin change interrupts will be enabled */

	_delay_ms(500);

	sei(); /* Enable interrupts */
}

ISR(PCINT0_vect)
{
	if( PORTB & (1<<CS) > 0 ) /*If true then CS pin has just gone high - SPI transaction finished */
	{
		USICR &= ~(1<<USIOIE); /* disble the overflow interrupt for the 4-bit counter */
	}
	else   /* Start of an SPI transaction */
	{
		USICR |= (1<<USIOIE);
		USISR = 1<<USIOIF;   /* resets the overflow flag - weirdly  */
	}

}

ISR(USI_OVF_vect)
{
	switch(USIDR)
	{
		case 65:
		USIDR= 5;
		break;

		case 66:
		USIDR= 7;
		break;

		default:
		USIDR=17;
		break;
	}
	USISR = 1<<USIOIF;	
}


int main()
{	
	setup_spi();
	for(;;);
	return 0;

}
