#define F_CPU 8000000UL
/* 8MHz causes garbled data  */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* 
References:

see: weathergadget.wordpress.com/2016/05/19/usi-spi-slave-communication/


*/

#define CS_SR	PB3 /*Shift Register*/
#define CS_ADC	PB4 /* MCP3208 */
#define DO	PB1
#define USCK	PB2
#define D1	PB0

uint8_t usi_clk_lo = (1<<USIWM0) | (1<<USITC);
uint8_t usi_clk_hi = (1<<USIWM0) | (1<<USITC) | (1<<USICLK);

uint8_t adc_buffer[3];

void setup_spi(void)
{
	DDRB |= 1<<DO;
	DDRB |= 1<<CS_SR;
	DDRB |= 1<<CS_ADC;
	DDRB |= 1<<USCK;

	/* To select software clock strobe USICS[1:0] = 0 */
	USICR = (1<<USIWM0); /* 3-wire mode and software clock  */
	
	PORTB |= 1<<CS_SR; /* Pull up resistor enabled on CS */
	PORTB |= 1<<CS_ADC;

	_delay_ms(500);

	sei(); /* Enable interrupts */
}

void spi_transfer_byte(uint8_t* bout, uint8_t* bin)
{
	int i = 0;
	USIDR = *bout;
	for(i=0;i<8;i++)
	{
		USICR = usi_clk_lo;
		USICR = usi_clk_hi;
	} 
	*bin = USIDR;
}

void spi_transfer_bytes(uint8_t* out, uint8_t* in, uint8_t n, uint8_t cs)
{
	PORTB &= ~(1<<cs);
	int i=0;
	for(i=0;i<n;i++)
	{
		spi_transfer_byte(out,in);
		out = out + 1;
		in = in + 1;
	}
	PORTB |= 1<<cs;
}

uint16_t get_adc_value(uint8_t chan)
{
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	
	spi_transfer_bytes(adc_buffer,adc_buffer,3,CS_ADC);
	
	uint16_t b1 = adc_buffer[1];
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

void display_value(uint8_t val)
{
	uint8_t dummy;
	spi_transfer_bytes(&val,&dummy,1,CS_SR);
}

int main()
{	
	setup_spi();
	uint16_t val = 0;
	uint8_t dval = 0;

	while(1)
	{
		val = get_adc_value(0);
		val = val >> 4;
		dval = val;
		display_value(dval);
		_delay_ms(100);
	}
	return 0;

}
