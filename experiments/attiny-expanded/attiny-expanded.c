#define F_CPU 8000000UL
/* Note: Fuses must be set to remove 8x clock divider */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* 

*/

#define CS_SR	PB3 /*Shift Register*/
#define CS_ADC	PB4 /* MCP3208 */
#define DO	PB1
#define USCK	PB2
#define D1	PB0


volatile uint8_t PORTSR = 0;
volatile uint8_t DUMMY = 0;

#define SRQA 0
#define SRQB 1
#define SRQC 2
#define SRQD 3
#define SRQE 4
#define SRQF 5
#define SRQG 6
#define SRQH 7

volatile uint8_t adc_buffer[3];




void spi_transfer_bytes(uint8_t* out, uint8_t* in, uint8_t n)
{
	int i=0;
	for(i=0;i<n;i++)
	{
		spi_transfer_byte(out,in);
		out = out + 1;
		in = in + 1;
	}
}

void spi_toggle_cs(uint8_t cs)
{
	PORTB ^= (1<<cs);
}

void spi_toggle_cs_expanded(uint8_t xcs)
{
	PORTSR ^= (1<<xcs);
	spi_toggle_cs(CS_SR); /*This is not going to work!*/
	
}


void display_value(uint8_t val)
{
	uint8_t dummy;
	spi_transfer_bytes(&val,&dummy,1,CS_SR);
}

