#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "mrf24jpi.h"
#include "pktspec.h"
#include "netspec.h"
#include "cmdspec.h"
#include "basenode.h"

// Which GPIO pin we're using

#define INT_PIN    0

// All nodes on the network, including the base unit, are identified by a unique 16 bit address. 
// The base unit (Rasperry Pi) is PI_BASE_ADDRESS
// The first node in the tree is NODE_BASE_ADDRESS
// In addition, each node has the same PANID which distinquishes members of this network from any others in range

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


// Current state of the pin
static volatile int state;
// Time of last change
struct timeval last_change;

uint16_t req_id;
volatile uint8_t keep_going;

// Requests sensor data from the specified node

void request_data(uint16_t target_node, uint16_t request_id)
{
    // Build the header - in this case the packet just consists of the addressing header and the command header
    uint8_t sz_packet = PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER;
    uint8_t packet[sz_packet];
    packet[PK_DEST_PANID_HI] = PAN_ID_HI;
    packet[PK_DEST_PANID_LO] = PAN_ID_LO;
    packet[PK_DEST_ADDR_HI] = (uint8_t)(target_node>>8);
    packet[PK_DEST_ADDR_LO] = (uint8_t)(255&target_node);
    packet[PK_SRC_PANID_HI] = PAN_ID_HI;
    packet[PK_SRC_PANID_LO] = PAN_ID_LO;
    packet[PK_SRC_ADDR_HI] = PI_ADDR_HI;
    packet[PK_SRC_ADDR_LO] = PI_ADDR_LO;
    packet[PK_COMMAND_HEADER+PK_HOP_COUNT] = 0; // HOP COUNTER - starts at zero
    packet[PK_COMMAND_HEADER+PK_SZ_PACKET] = sz_packet;
    packet[PK_COMMAND_HEADER+PK_CMD_HI] = 0x44;   //  0x4441 = DA from the base this is a data request
    packet[PK_COMMAND_HEADER+PK_CMD_LO] = 0x41;
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_0] = (uint8_t)(request_id>>8);  // This is an reference number for the request - should be unique to each request
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_1] = (uint8_t)(255&request_id);
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_2] = 1; // Data bitmask
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_3] = 0; 
	// Send the packet to the first node in the tree
	
	uint16_t cm0 = bytes_to_word(& packet[PK_COMMAND_HEADER+PK_CMD_HI]);
	printf("\nSENDING COMMAND: 0x%04X\n",cm0);
	
	
    mrf_send16(GATEWAY_NODE, packet, sz_packet);
}

void request_ping(uint16_t target_node, uint16_t request_id, uint8_t d)
{
	    // Build the header - in this case the packet just consists of the addressing header and the command header
    uint8_t sz_packet = PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER;
    uint8_t packet[sz_packet];
    packet[PK_DEST_PANID_HI] = PAN_ID_HI;
    packet[PK_DEST_PANID_LO] = PAN_ID_LO;
    packet[PK_DEST_ADDR_HI] = (uint8_t)(target_node>>8);
    packet[PK_DEST_ADDR_LO] = (uint8_t)(255&target_node);
    packet[PK_SRC_PANID_HI] = PAN_ID_HI;
    packet[PK_SRC_PANID_LO] = PAN_ID_LO;
    packet[PK_SRC_ADDR_HI] = PI_ADDR_HI;
    packet[PK_SRC_ADDR_LO] = PI_ADDR_LO;
    packet[PK_COMMAND_HEADER+PK_HOP_COUNT] = 0; // HOP COUNTER - starts at zero
    packet[PK_COMMAND_HEADER+PK_SZ_PACKET] = sz_packet;
    packet[PK_COMMAND_HEADER+PK_CMD_HI] = 0x45;   //  0x4441 = DA from the base this is a data request
    packet[PK_COMMAND_HEADER+PK_CMD_LO] = 0x52;
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_0] = (uint8_t)(request_id>>8);  // This is an reference number for the request - should be unique to each request
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_1] = (uint8_t)(255&request_id);
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_2] = d; 
    packet[PK_COMMAND_HEADER+PK_CMD_DATA_3] = 0; 
	// Send the packet to the first node in the tree
	
	uint16_t cm0 = bytes_to_word(& packet[PK_COMMAND_HEADER+PK_CMD_HI]);
	printf("\nSENDING COMMAND: 0x%04X\n",cm0);
	
	
    mrf_send16(GATEWAY_NODE, packet, sz_packet);
}

// This is called by the interrupt handler if new data is received

void handle_rx() {

	printf("\nRX\n");

    uint8_t * rx_data = mrf_get_rxdata(); // Pointer to the received data

    uint8_t sz_packet = rx_data[PK_SZ_CMD_HEADER+PK_SZ_PACKET]; // Size of teh received packet (see scheme above)

    uint8_t i = 0;
   
	
	
    if(bytes_to_word(& rx_data[PK_COMMAND_HEADER+PK_CMD_HI]) == CMD_DATA)  // Is this data coming back from the node?
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
  
    mrf_set_pan(PAN_ID);    // Set my panID
  // This is _our_ address
    mrf_address16_write(PI_ADDR);  // Set raspberry pi address
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
        printf("\nWaiting %i\n", req_id);
        request_ping(GATEWAY_NODE,req_id,0x42);
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
