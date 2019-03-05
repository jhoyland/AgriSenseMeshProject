#ifndef __PINDEFS_H
#define __PINDEFS_H

// Diagnostic LED pins

#define LED_PORT    PORTD	/* + */
#define LED_3       PD3    /*+REMOTE LED   GREEN*/
#define LED_2       PD4    /*+LOCAL LED 		YELLOW*/
#define LED_1		PD7		/*+ RED */


// MRF24J40 wifi pins


#define MRF_RESET   PB1		// --> Reset line
#define MRF_WAKE    PB6		// --> Wake line (for waking MRF)
#define MRF_INT     PD2		// <-- Interrupt line

#define MRF_RESET_PORT PORTB

// SPI Pins
#define SPI_MOSI    PB3		// --> SPI MOSI line
#define SPI_MISO    PB4		// <-- SPI MISO line
#define SPI_SCK     PB5		// --> SPI Clock line

// SPI Chip select lines
#define MRF_CS      PB2		// --> For wireless transceiver
#define ADC_CS 		PB7		// --> For ADC

#endif