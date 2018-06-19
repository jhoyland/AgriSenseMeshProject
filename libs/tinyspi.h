
#ifndef __TINYSPI_H
#define __TINYSPI_H

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

void spi_setup();
void spi_transfer_byte(uint8_t*,uint8_t*);
void spi_transfer_nbytes(uint8_t*,uint8_t*,uint8_t,uint8_t);

#endif



