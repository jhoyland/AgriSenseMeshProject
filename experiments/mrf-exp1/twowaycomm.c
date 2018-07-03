/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <tinyspi.h>
#include <mrf24j.h>
#include <blinkin.h>

#define MRF_RESET   PB1
#define MRF_CS      PB2
#define SPI_MOSI    PB3
#define SPI_MISO    PB4
#define SPI_SCK     PB5
#define MRF_WAKE    PB6

#define MRF_INT     PD2

#define LED_PORT    PORTD
#define LED_1       PD3    /*REMOTE LED*/
#define LED_2       PD4    /*LOCAL LED*/

#define BUTTON_1    PD5  
#define BUTTON_PIN  PIND5;


void setup() {

  /* Data directions */

  DDRB |= (1<<MRF_WAKE) | (1<<MRF_RESET) | (1<<MRF_CS);
  DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK);    

  DDRD |= (1<<LED_1) | (1<<LED_2);  

  mrf_reset();
  mrf_init();
  
  mrf_set_pan(0xcafe);
  // This is _our_ address
  mrf_address16_write(0x6001); 

  // uncomment if you want to receive any packet on this channel
  //mrf_set_promiscuous(true);
  
  // uncomment if you want to enable PA/LNA external control
  //mrf_set_palna(true);
  
  // uncomment if you want to buffer all PHY Payload
  //mrf_set_bufferPHY(true);

 // attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
 // last_time = millis();
  interrupts();
}

ISR(PCINT0_vect) {
    mrf_interrupt_handler(); // mrf24 object interrupt routine
}

uint8_t local_button_status;
uint8_t debounce_count;

void buttonChange()
{
	if(local_button_status == 1)
	{
		LED_PORT &= ~(1<<LED_1);
		local_button_status = 0;
	}
	else
	{
		LED_PORT |= 1<<LED_1;
		local_button_status = 1;
	}
}

uint8_t pollButton()
{
	if(BUTTON_PIN == 0) return 0;
		else return 1;
}

void loop() {
    mrf_check_flags(&handle_rx, &handle_tx);
    if ( debounce_count > 0 )
    {
		if( debounce_count == MAX_DEBOUNCE_COUNT )
		{
			buttonChange();
			debounce_count = 0;
		}
		
		debounce_count = debounce_count + 1;
	}
	else
	{
		if( pollButton() ^ local_button_status ) debounce_count = 1;
	}

        mrf_send16(0x4202, "abcd");
}
/*Called by check flags*/
void handle_rx() {
    //Serial.print("received a packet ");Serial.print(mrf_get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf_get_bufferPHY()){
      Serial.println("Packet data (PHY Payload):");
      for (int i = 0; i < mrf_get_rxinfo()->frame_length; i++) {
          Serial.print(mrf_get_rxbuf()[i]);
      }
    }
    
    Serial.println("\r\nASCII data (relevant data):");
    for (int i = 0; i < mrf_rx_datalength(); i++) {
        Serial.write(mrf_get_rxinfo()->rx_data[i]);
    }
    
    //Serial.print("\r\nLQI/RSSI=");
    //Serial.print(mrf_get_rxinfo()->lqi, DEC);
    //Serial.print("/");
    //Serial.println(mrf_get_rxinfo()->rssi, DEC);
}

void handle_tx() {
    if (mrf_get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf_get_txinfo()->retries);Serial.println(" retries\n");
    }
}


int main(void)
{
    setup();
    while(1) loop();

    return 0;
}
