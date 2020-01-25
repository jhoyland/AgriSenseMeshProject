#include <stdint.h>
#include "tinyspi.h"
//#define F_CPU 8000000UL

#define RED_LIGHT PD7
#define BLUE_LIGHT PD4
#define GREEN_LIGHT PD5

void spi_set_data_direction(uint8_t d)
{
    /*USI version of SPI can only do LSB first*/
    #ifndef SPI_ON_USI
    if(d == SPI_LSB)
        SPCR |=  (1 << DORD);
    else
        SPCR &= ~(1 << DORD);
    #else
   // #warning Data direction limited to LSB first for SPI on USI hardware
    #endif
}

void spi_setup()
{

	#ifndef SPI_ON_USI
	SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0); //enables SPI, sets chip as master, sets clock speed to fosc/16
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
	//PORTD |= (1<<GREEN_LIGHT);
	_delay_ms(100);
	DATAREG = *bout;
	while(! SPI_BYTE_XFER_DONE)
	{
		#ifdef SPI_ON_USI
		/*Toggle clock line*/
		USICR = usi_clk_lo;
		USICR = usi_clk_hi;
		#endif
	} 
	/*Incoming data out of data register*/
	*bin = DATAREG;
	//PORTD &= ~(1<<GREEN_LIGHT);
}

/*Selects the slave (cs)  and transfers n bytes. The input and output buffers must be defined and contain at least n bytes each*/

void spi_transfer_nbytes(uint8_t* out, uint8_t* in, uint8_t n, uint8_t cs)
{
	//DDRD |= (1<<BLUE_LIGHT);
	//DDRD |= (1<<RED_LIGHT);
	//DDRD |= (1<<GREEN_LIGHT);
	//PORTD |= (1<<BLUE_LIGHT);
	_delay_ms(1000);
	//PORTD &= ~(1<<BLUE_LIGHT);
	CS_PORT &= ~(1<<cs); /*Select slave chip*/
	while(n)
	{
		//PORTD |= (1<<RED_LIGHT);
		//_delay_ms(1000);
		//PORTD &= ~(1<<RED_LIGHT);
		//_delay_ms(1000);
		spi_transfer_byte(out,in);    /*transfer byte */
		//PORTD |= (1<<RED_LIGHT);
		//_delay_ms(1000);
		//PORTD &= ~(1<<RED_LIGHT);
		//_delay_ms(1000);
		/*advance iterators*/
		out = out + 1;
		in = in + 1;
		n = n - 1;
	}
	CS_PORT |= 1<<cs;/*Deselect slave chip*/
}
