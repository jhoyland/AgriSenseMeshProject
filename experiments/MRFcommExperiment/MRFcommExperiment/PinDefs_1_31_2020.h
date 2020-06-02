/*
 * PinDefs_1_31_2020.h
 *
 * Created: 1/31/2020 6:21:03 PM
 *  Author: Michael
 */ 


#ifndef PINDEFS_1_31_2020_H_
#define PINDEFS_1_31_2020_H_


#define MRF_INT     PD2		// <-- Interrupt line
#define MRF_WAKE    PD3		// --> Wake line (for waking MRF)
#define MRF_RESET   PD1		// --> Reset line

#define MRF_RESET_PORT PORTD

// SPI Pins
#define SPI_MOSI    PB3		// --> SPI MOSI line
#define SPI_MISO    PB4		// <-- SPI MISO line
#define SPI_SCK     PB5		// --> SPI Clock line

//ALL CS pins must be on PORTB
// SPI Chip select lines
#define MRF_CS      PB6		// --> For wireless transceiver //from PB2->PB6

//AVR DEFS

//Lights for diagnostics
#define LIGHT_PORT PORTD
#define BLUE_LIGHT PD4
#define RED_LIGHT PD7
#define YELLOW_LIGHT PD6
#define GREEN_LIGHT PD5

//Chip select lines
#define CS_SR	PB3 /*Shift Register*/ //MOSI line
#define ADC_CS	PB7 /* MCP3208 */

//required for ADC to work??
#define DO	PB1
//#define USCK PB2 //get rid of??
#define D1	PB0

//SPI Pins
//#define SPI_MOSI PB3
//#define SPI_MISO PB4
//#define SPI_SCK PB5
#define SPI_SS PB2



#endif /* PINDEFS_1_31_2020_H_ */