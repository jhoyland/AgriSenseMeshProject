#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "mrf24jpi.h"

// Which GPIO pin we're using

#define INT_PIN    0

// All nodes on the network, including the base unit, are identified by a unique 16 bit address. 
// The base unit (Rasperry Pi) is PI_BASE_ADDRESS
// The first node in the tree is NODE_BASE_ADDRESS
// In addition, each node has the same PANID which distinquishes members of this network from any others in range

#define PI_BASE_ADDRESS 0x3142		
#define NODE_BASE_ADDRESS 0x1010
#define PANID 0xf122

// In this scheme each transmission is accompanied by 14 bytes of data (this forms part of the data payload
// of the transmission. The transceivers also prepend data required by the communication protocol. The user does not 
// need to deal with these as it is handled automatically. The protocol header can be accessed though if required.
// In our scheme the 14 bytes are
/* 
0	target panID hi byte
1	target panID lo byte
2	target node address hi byte
3	target node address lo byte
4	source panID hi byte
5	source panID lo byte
6	source node address hi byte
7	source node address lo byte
8	hop counter (this stores how many nodes the message gets passed through before it reaches the target)
9	the length of the data packet (including this header)
10	command hi byte   the command word indicates the content of the message
11	command lo byte
12	command data hi byte optional data associated with the command (e.g. arguments)
13	command data lo byte
*/


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

// Requests sensor data from the specified node

void request_data(uint16_t target_node, uint16_t request_id)
{
    // Build the header - in this case the packet just consists of the addressing header and the command header
    uint8_t sz_packet = SZ_ADDRESSING_HEADER + SZ_PKT_CMD;
    uint8_t packet[sz_packet];
    packet[0] = 0xF1;
    packet[1] = 0x22;
    packet[2] = (uint8_t)(target_node>>8);
    packet[3] = (uint8_t)(255&target_node);
    packet[4] = 0xF1;
    packet[5] = 0x22;
    packet[6] = 0x31;
    packet[7] = 0x42;
    packet[8] = 0; // HOP COUNTER - starts at zero
    packet[9] = sz_packet;
    packet[10] = 0x44;   //  0x4441 = DA from the base this is a data request
    packet[11] = 0x41;
    packet[12] = (uint8_t)(request_id>>8);  // This is an reference number for the request - should be unique to each request
    packet[13] = (uint8_t)(255&request_id);
	// Send the packet to the first node in the tree
    mrf_send16(NODE_BASE_ADDRESS, packet, sz_packet);
}

// This is called by the interrupt handler if new data is received

void handle_rx() {

    // We are not interested in the full "physical" buffer - this includes the automatic data
    // prepended by the traciever - if we want to do something with it we can capture it here
    if(mrf_get_bufferPHY()){

    }

    uint8_t * rx_data = mrf_get_rxdata(); // Pointer to the received data

    uint8_t sz_packet = rx_data[9]; // Size of teh received packet (see scheme above)

    uint8_t i = 0;
    
	
	
    if(bytes_to_word(& rx_data[10]) == CMD_DATA)  // Is this data coming back from the node?
    {

		printf("\nRX:\n==========\n\n");
		
	       // In this test we are receiving a single pair or 12-bit data items
	    	// Representing the mean and std error of the sensor reading
	    	// These are packed into 3 bytes by the node so need unpacking here
	    
		uint16_t data[2];
		
		unpack_12bit(data,& rx_data[14]);
		
		printf("\nRequest no %d",bytes_to_word(& rx_data[12]));
		printf("\nMean = %d   stderr = %d",data[0],data[1]);
		

		printf("\n===END====\n\n");

	}
	else // At the moment we are not expecting anything else! Could have nodes send
		// diagnostic data or error messages etc though
	{
		printf("\nUrecognized packet");
	}
}

// This is called by the interrupt handler when transmit has completed and the receiver has acknowledged
// The acknowledgement is automatic however, note it is the actual receiver of this message which sends
// acknowledge - not the final destination node. In this case all it tells us is if the BASE_NODE of the 
// network has received the message

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
    pullUpDnControl(INT_PIN, PUD_UP); /*Interrupt pin must idle high*/
    pullUpDnControl(RESET_PIN, PUD_UP); /*Reset pin must idle high*/

	// Time now
	gettimeofday(&last_change, NULL);

	// Bind to interrupt - this allows the RF module to interrupt the Pi's processor when a RX or TX happen
	wiringPiISR(INT_PIN, INT_EDGE_FALLING, &mrf_interrupt_handler);

    mrf_reset(); // Reset the MRF chip
    mrf_init();  // Initialize the MRF  chip
  
    mrf_set_pan(0xf122);    // Set my panID
  // This is _our_ address
    mrf_address16_write(0x3142);  // Set raspberry pi address
// some loop flags for this experiment
		
    keep_going = 8;
    req_id = 1;

}

void loop() {
	
	// Check if any new interrupts have triggered
    mrf_check_flags(&handle_rx, &handle_tx);

    struct timeval new_time;
    gettimeofday(&new_time,NULL);

	// request data every 5 seconds
	
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
