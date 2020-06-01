/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */

/*#define F_CPU 1000000UL*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "tinyspi.h"
#include "mrf24j.h"
#include "blinkin.h"
#include "mrfpindefs.h"
#include "bitmanip.h"


#define PI_ADDRESS 0x3142

#define LED_PORT    PORTD
#define LED_1       PD3    /*REMOTE LED*/
#define LED_2       PD4    /*LOCAL LED*/

#define ADC_CS PB7
#define CMD_DATA 0x4441     // DA
#define CMD_ERROR 0x4552    // ER
#define CMD_BYTE 0x5342     // SB

#define NS_DATA_REQUESTED 1
#define NS_DATA_READY 2
#define NS_ERROR 3
#define NS_MEASURING 4

#define NX_UNREC_CMD 1
#define NX_WRONG_SRC 2

#define ADDRESS_HI (MY_ADDRESS >> 8)
#define ADDRESS_LO (255 & MY_ADDRESS)

#define PI_ADDRESS_HI (PI_ADDRESS >> 8)
#define PI_ADDRESS_LO (255 & PI_ADDRESS)

#define PANID_HI (ASMP_PANID >> 8)
#define PANID_LO (255 & ASMP_PANID)

#define SZ_ADDRESSING_HEADER 8
#define SZ_PKT_CMD 6

#define SAMPLING_DELAY 5 
#define N_ADC_CHANS 1
#define SZ_RAW_DATA_BUFFER 8

uint16_t last_data_request;
uint8_t status;
uint8_t errcode;
uint16_t loop_counter;


uint8_t current_rx_data[aMaxPHYPacketSize];
uint8_t packet[aMaxPHYPacketSize];

uint8_t adc_buffer[3];

uint16_t raw_data_buffer[SZ_RAW_DATA_BUFFER];
uint8_t data_package[3 * N_ADC_CHANS];



/* ADC FUNCTIONS */


uint16_t get_adc_value(uint8_t chan)
{
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	
	spi_transfer_nbytes(adc_buffer,adc_buffer,3,ADC_CS);
	
	uint16_t b1 = adc_buffer[1];
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

void get_raw_data()
{
    status |= (1 << NS_MEASURING);
    int ch = 0;
    uint16_t measurement[2];
    for(ch = 0; ch<N_ADC_CHANS; ch++)
    {
        int i = 0;
        measurement[0] = 0;
        for(i = 0; i<SZ_RAW_DATA_BUFFER; i++)
        {
            raw_data_buffer[i] = get_adc_value(ch);
            measurement[0] += raw_data_buffer[i];
            _delay_ms(SAMPLING_DELAY);
        }
        measurement[0] /= SZ_RAW_DATA_BUFFER;
        measurement[1] = 0;
        uint16_t dev;
        for(i = 0; i<SZ_RAW_DATA_BUFFER; i++)
        {
            dev = raw_data_buffer[i] > measurement[0] ? raw_data_buffer[i] - measurement[0] : measurement[0] - raw_data_buffer[i];
            measurement[1] += dev * dev;   
        }
        measurement[1] = sqrt(measurement[1] / (SZ_RAW_DATA_BUFFER * (SZ_RAW_DATA_BUFFER-1)));  //Standard error

        pack_12bit(& data_package[3*ch], measurement);
    }
    status &= ~(1<<NS_MEASURING);
    status |= (1<<NS_DATA_READY);
}


void send_packet_in(uint8_t sz_packet)
{
    uint8_t next_address[2];
    next_address[0] = ADDRESS_HI;
    next_address[1] = ADDRESS_LO;

    if(next_address[0] == 0x10) // I am a row coordinator
    {
        if(next_address[1] == 0x10) // I am the first node (send direct to Raspberry Pi
        {
            next_address[0] = PI_ADDRESS_HI;
            next_address[1] = PI_ADDRESS_LO;
        }
        else
        {
            next_address[1] = next_address[1] - 1;
        }

    }
    else
    {
        next_address[0] = next_address[0] - 1;
    }

    mrf_send16(bytes_to_word(next_address),packet,sz_packet);
}

void send_byte(uint8_t b)
{
    uint8_t sz_packet = SZ_ADDRESSING_HEADER + SZ_PKT_CMD + 1;
   // uint8_t packet[sz_packet];
    packet[0] = PANID_HI;
    packet[1] = PANID_LO;
    packet[2] = PI_ADDRESS_HI;
    packet[3] = PI_ADDRESS_LO;
    packet[4] = PANID_HI;
    packet[5] = PANID_LO;
    packet[6] = ADDRESS_HI;
    packet[7] = ADDRESS_LO;
    packet[8] = 0; // HOP COUNTER
    packet[9] = sz_packet;
    packet[10] = 0x53;
    packet[11] = 0x42;
    packet[12] = 0;
    packet[13] = 0;
    packet[14] = b;

    send_packet_in(sz_packet);

}

void send_data()
{
    uint8_t sz_packet = SZ_ADDRESSING_HEADER + SZ_PKT_CMD + (3*N_ADC_CHANS);
   // uint8_t packet[sz_packet];
    packet[0] = PANID_HI;
    packet[1] = PANID_LO;
    packet[2] = PI_ADDRESS_HI;
    packet[3] = PI_ADDRESS_LO;
    packet[4] = PANID_HI;
    packet[5] = PANID_LO;
    packet[6] = ADDRESS_HI;
    packet[7] = ADDRESS_LO;
    packet[8] = 0; // HOP COUNTER
    packet[9] = sz_packet;
    packet[10] = 0x44;
    packet[11] = 0x41;
    if(status | (1<<NS_DATA_REQUESTED))
    {
        packet[12]= 255 & (last_data_request >> 8);  // If this is requested data will send id of last request. Sends zero if this is auto-data
        packet[13]= 255 & last_data_request;
    }
    else
    {
        packet[12] = 0;
        packet[13] = 0;
    }

    int i = 0;
    
    for(i = 0; i<3*N_ADC_CHANS; i++)
    {
        packet[14+i] = data_package[i];
    }

    send_packet_in(sz_packet);

    status &= ~(1<<NS_DATA_READY);
    if(status | (1<<NS_DATA_REQUESTED)) status &= ~(1<<NS_DATA_REQUESTED);
}



/*Called by check flags*/
void handle_rx() {

    
    if(mrf_get_bufferPHY()){

    }
	uint8_t * rx_data = mrf_get_rxdata();
    memcpy(current_rx_data,rx_data,aMaxPHYPacketSize);

	//_delay_ms(100);
	
    if(bytes_to_word(& current_rx_data[2]) == MY_ADDRESS) // Is for me
    {
		
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_1);
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_1);
		
        if(bytes_to_word(& current_rx_data[6]) == PI_ADDRESS)  // Is from the Pi (shouldn't be from anywhere else!)
        {
            if(bytes_to_word(& current_rx_data[10]) == CMD_DATA)
            {
                status |= (1<<NS_DATA_REQUESTED);
                last_data_request = bytes_to_word(& current_rx_data[12]);

                get_raw_data();
                send_data();
                
                
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_2);
            }
            else
            {
                status |= (1<<NS_ERROR);
                errcode |= (1<<NX_UNREC_CMD);
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_2);

                send_byte(1);
            }
        }
        else
        {
            send_byte(2);
            status |= (1<<NS_ERROR);
            errcode |= (1<<NX_WRONG_SRC);
        }
    }
    else
    {
        send_byte(3);
        if(current_rx_data[3] == 0 && current_rx_data[4] == 0) // Is for everyone
        {
            
        }
        
      //  relay_packet();
      
      
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_1);
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_1);

    }

}

void handle_tx() {

	
    if (mrf_tx_ok()) {
        BLINK(LED_PORT,LED_2);
        BLINK(LED_PORT,LED_1);
        BLINK(LED_PORT,LED_2);
    } else {
        BLINK(LED_PORT,LED_2);
    }
}

void setup() {

  /* Data directions */

  DDRB |= (1<<MRF_WAKE) | (1<<MRF_RESET) | (1<<MRF_CS);
  DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK); 
  DDRB |= (1<<ADC_CS);   

  DDRD |= (1<<LED_1) | (1<<LED_2);  
  
//  PORTD |= (1<<BUTTON_1);
  PORTB |= (1<<MRF_CS); //
  PORTB |= (1<<ADC_CS);
  PORTD |= (1<<MRF_INT);
  MRF_RESET_PORT |= (1<<MRF_RESET);
  spi_set_data_direction(SPI_MSB);
  spi_setup();

  mrf_reset();
  mrf_init();
  
  mrf_set_pan(ASMP_PANID);
  // This is _our_ address
  mrf_address16_write(MY_ADDRESS); 
  
  loop_counter = 0;

  // uncomment if you want to receive any packet on this channel
  //mrf_set_promiscuous(true);
  
  // uncomment if you want to enable PA/LNA external control
  //mrf_set_palna(true);
  
  // uncomment if you want to buffer all PHY Payload
  //mrf_set_bufferPHY(true);

 // attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
 // last_time = millis();
  sei();
  EIMSK |= (1<<INT0);
  EICRA |= (1<<ISC01);
  
}

ISR(INT0_vect) {
    mrf_interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
    mrf_check_flags(&handle_rx, &handle_tx);

		if( loop_counter > 32768 )
		{
			loop_counter = 0;
			BLINK(LED_PORT,LED_1);
			BLINK(LED_PORT,LED_1);
			BLINK(LED_PORT,LED_1);
			BLINK(LED_PORT,LED_1);
		//	char* msg = "aaaa";
			//mrf_send16(PI_ADDRESS, (uint8_t*)msg, 4);
        //    send_data();
		}
		
		loop_counter = loop_counter + 1;
			
}



int main(void)
{
    setup();
    while(1) loop();

    return 0;
}
