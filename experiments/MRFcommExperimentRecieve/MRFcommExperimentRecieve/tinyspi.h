
#ifndef __TINYSPI_H
#define __TINYSPI_H

#include <avr/io.h>
//#include <util/delay.h>
//#include <avr/interrupt.h>

#define SZ_XFER_BUFF 128

//#define DO PB3
//#define USCK PB5

#ifndef SPI_CS_PORT
#define SPI_CS_PORT PORTB
#endif

//#define SPI_ON_USI

#ifndef SPI_ON_USI

#define DATAREG SPDR
#define STATUSREG SPSR
#define XFER_DONE_FLAG SPIF

#else /* SPI_ON_USI is defined for chips lacking true SPI hardware such as ATTiny85 */

#define DATAREG USIDR
#define STATUSREG USISR
#define XFER_DONE_FLAG USIOIF

#define usi_clk_lo (1<<USIWM0) | (1<<USITC);
#define usi_clk_hi (1<<USIWM0) | (1<<USITC) | (1<<USICLK);

#endif

#define SPI_BYTE_XFER_DONE (STATUSREG & (1<<XFER_DONE_FLAG))

#define SCK_F_4 0			// 00
#define SCK_F_16 1			// 01
#define SCK_F_64 2			// 10
#define SCK_F_128 3			// 11

#define SCK_F_2 4
#define SCK_F_8 5
#define SCK_F_32 6

#define SPI_X2 1
#define SPI_X1 0

#define SPI_LSB 0
#define SPI_MSB 1

void spi_set_data_direction(uint8_t);
void spi_setup();
void spi_transfer_byte(uint8_t*,uint8_t*);
void spi_transfer_nbytes(uint8_t*,uint8_t*,uint8_t,uint8_t);
void spi_set_speed(uint8_t);

#endif



