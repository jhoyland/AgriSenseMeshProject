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
	DDRB |= 1<<CS;
	DDRB |= 1<<USCK;

	/* To select software clock strobe USICS[1:0] = 0 */
	USICR = (1<<USIWM0); /* 3-wire mode and software clock  */
	
	PORTB |= 1<<CS; /* Pull up resistor enabled on CS */

	_delay_ms(500);

	sei(); /* Enable interrupts */
}

int main()
{	
	setup_spi();

	uint8_t mydata = 0;

	uint8_t i = 0;
	uint8_t j = 0;
/* This seems to be the best way of toggling the USI clock. Just writing to USITC
Seems to mess up the other settings causing it to come out of three wire mode  */
	uint8_t usi_clk_lo = (1<<USIWM0) | (1<<USITC);
	uint8_t usi_clk_hi = (1<<USIWM0) | (1<<USITC) | (1<<USICLK);

	while(1)
	{
		mydata = mydata + 1;
		USIDR = mydata;  /*Load data byte into data register*/

		PORTB &= ~(1<<CS); /*Pull chip select line low*/

		for(i=0;i<8;i++)  /*Clock out data byte*/
		{
			USICR = usi_clk_lo;
			USICR = usi_clk_hi;
		}

		PORTB |= 1<<CS; /*Pull chip select line high*/

		_delay_ms(500);

		j = j+1;
		if(j==8) j=0;
		if(mydata > 200) mydata = 0;
	}
	return 0;

}
