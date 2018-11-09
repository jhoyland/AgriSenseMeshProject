#define F_CPU 8000000UL
/* Note: Fuses must be set to remove 8x clock divider */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "spi-master-adc1.h"

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



uint16_t get_adc_value(uint8_t chan)
{
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	
	spi_transfer_bytes(adc_buffer,adc_buffer,3);
	
	uint16_t b1 = adc_buffer[1];
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

void display_value(uint8_t val)
{
	uint8_t dummy;
	spi_transfer_bytes(&val,&dummy,1);
}

