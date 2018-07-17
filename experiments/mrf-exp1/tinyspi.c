#include <stdint.h>
#include "tinyspi.h"



void spi_set_data_direction(uint8_t d)
{
    /*USI version of SPI can only do LSB first*/
    #ifndef SPI_ON_USI
    if(d == SPI_LSB)
        SPCR |=  (1 << DORD);
    else
        SPCR &= ~(1 << DORD);
    #else
    #warning Data direction limited to LSB first for SPI on USI hardware
    #endif
}

void spi_setup()
{

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
	CS_PORT &= ~(1<<cs); /*Select slave chip*/
	while(n)
	{
		spi_transfer_byte(out,in);    /*transfer byte */
		/*advance iterators*/
		out = out + 1;
		in = in + 1;
		n = n - 1;
	}
	CS_PORT |= 1<<cs;/*Deselect slave chip*/
}
