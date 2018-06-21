#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SZ_XFER_BUFF 128

//#define DO PB3
//#define USCK PB5


#ifndef SPI_ON_USI

#define DATAREG SPDR
#define STATUSREG SPSR
#define XFER_DONE_FLAG SPIF

#else /* SPI_ON_USI is defined for chips lacking true SPI hardware such as ATTiny85 */

#define DATAREG USIDR
#define SATUSREG USISR
#define XFER_DONE_FLAG USIOIF

#define usi_clk_lo (1<<USIWM0) | (1<<USITC);
#define usi_clk_hi (1<<USIWM0) | (1<<USITC) | (1<<USICLK);

#endif

#define SPI_BYTE_XFER_DONE (STATUSREG & (1<<XFER_DONE_FLAG))

#define SCK_F_4 0
#define SCK_F_16 1
#define SCK_F_64 2
#define SCK_F_128 3

#define SPI_X2 1
#define SPI_X1 0

#ifndef 



#endif


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
	DATAREG = *bout; /* SPI transfer initiated by write to data register for SPI hardware - not for */

	while(! SPI_BYTE_XFER_DONE)
	{
		#ifdef SPI_ON_USI
		/*Toggle clock line. Not needed for ATMega and other MCU's with true SPI hardware */
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
	CSPORT &= ~(1<<cs); /*Select slave chip*/
	while(n)
	{
		spi_transfer_byte(out,in);    /*transfer byte */
		/*advance iterators*/
		out = out + 1;
		in = in + 1;
		n = n - 1;
	}
	CSPORT |= 1<<cs;/*Deselect slave chip*/
}
