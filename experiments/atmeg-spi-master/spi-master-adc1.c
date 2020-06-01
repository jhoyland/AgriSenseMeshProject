#define F_CPU 8000000UL
/* 8MHz causes garbled data  */

#include "../../libs/tinyspi.h"

/* 
References:

see: weathergadget.wordpress.com/2016/05/19/usi-spi-slave-communication/


*/

#define CS_ADC	PB2 /* MCP3208 */


void spi_setup()
{
	/*DDRB |= 1<<data_out_pin;
	DDRB |= 1<<clock_pin;*/

	#ifndef SPI_ON_USI
	SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0);
	#else
	/* To select software clock strobe USICS[1:0] = 0 */ 
	USICR = (1<<USIWM0); /* 3-wire mode and software clock  */
	#endif
	/*
	Chip select lines must also be set as outputs and pull up resistors enabled
	*/
}

/*Transfers a single byte between master and slave*/

void spi_transfer_byte(uint8_t* bout, uint8_t* bin)
{
	/*Outgoing data into data register*/
	DATAREG = *bout;

	while(! SPI_BYTE_XFER_DONE)
	{
		#ifdef SPI_ON_USI
		/*Toggle clock line*/
		USICR = usi_clk_lo;
		USICR = usi_clk_hi;
		#endif
	} 
	/*Incomming data out of data register*/
	*bin = DATAREG;
}

/*Selects the slave (cs)  and transfers n bytes. The input and output buffers must be defined and contain at least n bytes each*/

void spi_transfer_nbytes(uint8_t* out, uint8_t* in, uint8_t n, uint8_t cs)
{
	PORTB &= ~(1<<cs); /*Select slave chip*/
	while(n)
	{
		spi_transfer_byte(out,in);    /*transfer byte */
		/*advance iterators*/
		out = out + 1;
		in = in + 1;
		n = n - 1;
	}
	PORTB |= 1<<cs;/*Deselect slave chip*/
}

uint8_t adc_buffer[3];


uint16_t get_adc_value(uint8_t chan)
{
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	
	spi_transfer_nbytes(adc_buffer,adc_buffer,3,CS_ADC);
	
	uint16_t b1 = adc_buffer[1];
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

int main()
{	

    DDRB |= (1<<PB3)|(1<<PB5)|(1<<CS_ADC); 
    DDRD |= (1<<PD2);
    PORTB |= (1<<CS_ADC);

	spi_setup();


	uint16_t val = 0;
	uint16_t dval = 0;

	while(1)
	{
		val = get_adc_value(0);

      

        if(val > 1000) 
            PORTD |= (1<<PD2);
        else
            PORTD &= ~(1<<PD2);

		_delay_ms(100);
	}
	return 0;

}
