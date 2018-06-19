


#include "tinyspi.h"

/*#define DO		PB1
#define USCK	PB2*/


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
