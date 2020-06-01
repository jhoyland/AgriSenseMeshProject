#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "mrf24jpi.h"

// Which GPIO pin we're using

#define INT_PIN    0

#define PI_BASE_ADDRESS 0x3142
#define NODE_BASE_ADDRESS 0x1010


// Current state of the pin
static volatile int state;
// Time of last change
struct timeval last_change;

uint8_t led_counter;
volatile uint8_t keep_going;

void handle_rx() {

    
    if(mrf_get_bufferPHY()){

    }

	uint8_t * rx_data = mrf_get_rxdata();

    printf("\nReceived: %s\n", rx_data);


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

	// Get initial state of pin
	state = digitalRead(INT_PIN);

	if (state) {
		printf("Started! Initial state is on\n");
	}

    mrf_reset();
    mrf_init();
  
    mrf_set_pan(0xcafe);
  // This is _our_ address
    mrf_address16_write(0x3142); 

    led_counter = 1;
    keep_going = 4;
}

void loop() {
    mrf_check_flags(&handle_rx, &handle_tx);

    struct timeval new_time;

    gettimeofday(&new_time,NULL);

    if( (new_time.tv_sec - last_change.tv_sec) > 5 )
    {
        printf("Sending: %i\n", led_counter);
		mrf_send16(NODE_BASE_ADDRESS, &led_counter, 1);
        led_counter = led_counter + 1;
        if(led_counter > 4) led_counter = 1;
        last_change = new_time; 
        keep_going = keep_going - 1;
    } 

                     
}

int main(void)
{
    setup();
    while(keep_going) loop();
    return 0;   
}
