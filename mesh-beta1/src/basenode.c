#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "mrf24jpi.h"

// Which GPIO pin we're using

#define INT_PIN    0

#define PI_BASE_ADDRESS 0x3142
#define NODE_BASE_ADDRESS 0x1010
#define PANID 0xf122


#define SZ_ADDRESSING_HEADER 8
#define SZ_PKT_CMD 6


// Current state of the pin
static volatile int state;
// Time of last change
struct timeval last_change;

uint16_t req_id;
volatile uint8_t keep_going;

#define FIRST_NODE 0x1010

#define CMD_DATA 0x4441     // DA
#define CMD_ERROR 0x4552    // ER
#define CMD_BYTE 0x5342     // SB

void request_data(uint16_t request_id)
{
    uint8_t sz_packet = SZ_ADDRESSING_HEADER + SZ_PKT_CMD;
    uint8_t packet[sz_packet];
    packet[0] = 0xF1;
    packet[1] = 0x22;
    packet[2] = 0x10;
    packet[3] = 0x10;
    packet[4] = 0xF1;
    packet[5] = 0x22;
    packet[6] = 0x31;
    packet[7] = 0x42;
    packet[8] = 0; // HOP COUNTER
    packet[9] = sz_packet;
    packet[10] = 0x44;
    packet[11] = 0x41;
    packet[12] = (uint8_t)(request_id>>8);
    packet[13] = (uint8_t)(255&request_id);

    mrf_send16(NODE_BASE_ADDRESS, packet, sz_packet);
}

void handle_rx() {

    
    if(mrf_get_bufferPHY()){

    }

	uint8_t * rx_data = mrf_get_rxdata();

    uint8_t sz_packet = rx_data[9];

    uint8_t i = 0;
    
    if(bytes_to_word(& rx_data[10]) == CMD_DATA)
    {

		printf("\nRX:\n==========\n\n");

		uint16_t data[2];
		
		unpack_12bit(data,& rx_data[14]);
		
		printf("\nRequest no %d",bytes_to_word(& rx_data[12]));
		printf("\nMean = %d   stderr = %d",data[0],data[1]);
		

		printf("\n===END====\n\n");

	}
	else
	{
		printf("\nUrecognized packet");
	}
}

void handle_tx() {

	
    if (mrf_tx_ok()) {
        printf("\nTransmit acknowledged\n");
    } else {
        printf("\nTransmit failed\n");
    }
}

void setup() {
    wiringPiSetup();

	// Set pin to output in case it's not
	pinMode(INT_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    pullUpDnControl(INT_PIN, PUD_UP);
    pullUpDnControl(RESET_PIN, PUD_UP); /*Reset pin must idle high*/

	// Time now
	gettimeofday(&last_change, NULL);

	// Bind to interrupt
	wiringPiISR(INT_PIN, INT_EDGE_FALLING, &mrf_interrupt_handler);

    mrf_reset();
    mrf_init();
  
    mrf_set_pan(0xf122);
  // This is _our_ address
    mrf_address16_write(0x3142); 

    keep_going = 8;
    req_id = 1;

}

void loop() {
    mrf_check_flags(&handle_rx, &handle_tx);

    struct timeval new_time;
    gettimeofday(&new_time,NULL);

    if( (new_time.tv_sec - last_change.tv_sec) > 5 )
    {
        printf("\nRequesting: %i\n", req_id);
        request_data(req_id);
        last_change = new_time; 
        keep_going = keep_going - 1;
        req_id ++;
    } 

                     
}

int main(void)
{
    setup();
    while(keep_going) loop();
    return 0;   
}
