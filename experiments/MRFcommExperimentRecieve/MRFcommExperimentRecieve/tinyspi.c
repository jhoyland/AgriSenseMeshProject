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

// The lowest 2bits of SPCR set the speed. This function sets them to the user suppled value
// If the user supplies a value >3 the speed is set by by the bottom 2 bits of the supplied value


void spi_set_speed(uint8_t spd)
{
	uint8_t spbits = 7 & spd; // Suppress bits above bit 3 (just makes sure user doesn't provide invalid bits)

	SPCR &= ~3;  // Clear speed bits (sets speed to default = fosc / 4)

	SPCR |= (3 & spbits);  // Set user bits

	if(spbits & 4) SPSR |= 1;
		else SPSR &= (~1);
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
	SPI_CS_PORT &= ~(1<<cs); /*Select slave chip*/
	while(n)
	{
		spi_transfer_byte(out,in);    /*transfer byte */
		/*advance iterators*/
		out = out + 1;
		in = in + 1;
		n = n - 1;
	}
	SPI_CS_PORT |= 1<<cs;/*Deselect slave chip*/
}
