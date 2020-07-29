/* This program will simply send out requests, and listen for data being exchanged by the nodes
It does this by looking for packets on the same PANID (0xCAFE) */

#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <string.h>
#include "simple_queue.h"
#include "mrf24jpi.h"
#include "pktspec.h"
#include "netspec.h"
#include "cmdspec.h"
#include "bitmanip.h"
#include "setup.h"

//which GPIO pin we're using
#define INT_PIN 0 //the interrupt pin

uint8_t req_id; //formerly 16
uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t received_data_buffer[PK_SZ_TXRX_BUFFER];

uint8_t error_data_buffer[PK_SZ_ERR_BUFFER];
uint8_t* transmit_command_header;
uint8_t active_message[PK_SZ_TXRX_BUFFER];
uint8_t running_status = 0;
uint8_t got_message;


void handle_rx() {
    printf("\nEntering Handle_rx (setup)"); fflush(stdout);

    running_status |= (1<<RU_RX_HANDLE);
  
    //mrf_rx_disable(); //suggested by james

    //uint8_t buffer_length = mrf_rx_datalength();//mrf_rx_buffer_datalength(); // formerly mrf_rx_datalength(); 
    //printf("\nbuffer_length (setup) %i",buffer_length); fflush(stdout);
    //memcpy(received_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()); // Copy the message into the recieved data buffer
    //printf("\nmrf_rx_buffer_datalength(setup): %i",buffer_length);
    /*int i = 0;
	for(i = 0; i < buffer_length; i++)
	{
		printf("\ngot: %i 0x %x", i, received_data_buffer[i]);
		fflush(stdout);
	}*/	
    
   mrf_rx_enable();
   running_status &=~(1<<RU_RX_HANDLE);
   printf("\nExiting handle_rx"); fflush(stdout);
   //print_received_message(); fflush(stdout); //segmentation fault if processed here

}

void print_received_message()
{
	uint8_t buffer_length = mrf_rx_buffer_datalength();
	printf("buffer length:(PM) %i" , buffer_length);
	uint8_t received_data_buffer[buffer_length];
	memcpy(received_data_buffer,mrf_get_rx_data_buffer(),buffer_length); // Copy the message into the recieved data buffer
 	int i = 0;
	if(buffer_length > 50) {printf("\nIncorrect Message"); fflush(stdout);}
	else{
		for(i = 0; i < buffer_length; i++)
		{
			printf("\ngot(PM): %i 0x %x", i, received_data_buffer[i]);
			fflush(stdout);
		}
	     } 
}

// This is called by the interrupt handler when transmit has completed and the receiver has acknowledged
// The acknowledgement is automatic however, note it is the actual receiver of this message which sends
// acknowledge - not the final destination node. In this case all it tells us is if the BASE_NODE of the 
// network has received the message

void handle_tx() {

   // tx_info_t * inf;

    //mrf_get_txinfo(&inf);

   // printf("\nStatus register:%x  &0x3F = %x     ! = %d",mrf_reg_TXSTAT,mrf_reg_TXSTAT & 0x3F,!(mrf_reg_TXSTAT & 0x3F));
	//fflush(stdout);
//    if (!(mrf_reg_TXSTAT & 0x3F)/*mrf_tx_ok()*/) {
      //  printf("\nTX ACK");
  //  } else {
    //    printf("\nTX FAIL");
//    }
}

void set_packet_header(uint8_t* buff)
{

    //buff[PK_DEST_PANID_HI] = PAN_ID_HI; //no longer needed bits
    //buff[PK_DEST_PANID_LO] = PAN_ID_LO;
    //buff[PK_SRC_PANID_HI] = PAN_ID_HI;
    //buff[PK_SRC_PANID_LO] = PAN_ID_LO;
    buff[PK_SRC_ADDR_HI] = PI_ADDR_HI;
    buff[PK_SRC_ADDR_LO] = PI_ADDR_LO;
    buff[PK_COMMAND_HEADER+PK_HOP_COUNT] = 0;

}

void setup()
{
 	//setup_queue(&message_queue,SZ_MESSAGE_QUEUE,SZ_MESSAGE,0);
    	transmit_command_header = &(transmit_data_buffer[PK_COMMAND_HEADER]);

    	wiringPiSetup();
    	wiringPiSPISetup (0, 1000000) ;

	// Set pin to output in case it's not
    	pinMode(INT_PIN, INPUT);   // **JAMES I changed this to INPUT - could be firing interrupt randomly
    	pinMode(RESET_PIN, OUTPUT);
    	pullUpDnControl(INT_PIN, PUD_UP); /*Interrupt pin must idle high*/
    	pullUpDnControl(RESET_PIN, PUD_UP); /*Reset pin must idle high*/

	// Time now
	//gettimeofday(&last_change, NULL);

	// Bind to interrupt - this allows the RF module to interrupt the Pi's processor when a RX or TX happen
	wiringPiISR(INT_PIN, INT_EDGE_FALLING, &mrf_interrupt_handler);

    	mrf_reset(); // Reset the MRF chip
    	mrf_init();  // Initialize the MRF  chip
  
    	mrf_set_pan(PAN_ID);    // Set my panID
  	// This is _our_ address
    	mrf_address16_write(PI_ADDR);  // Set raspberry pi address
	// some loop flags for this experiment
		
   	// keep_going = 8;
  	req_id = 1;

    	set_packet_header(transmit_data_buffer);

}


//sending a message requires 4 operations:
//Setting the packet size
//Setting the Target Node
//Setting a command
//Setting a request ID

void set_packet_size(uint8_t* buff, uint8_t sz)
{

    buff[PK_COMMAND_HEADER + PK_SZ_PACKET] = sz;
}

void set_target_node(uint8_t* buff,uint16_t target_node)
{ 
    word_to_bytes(& buff[PK_DEST_ADDR_HI], target_node);
}

void set_command(uint8_t* buff, uint16_t cmd_id, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4)
{
    word_to_bytes(& buff[PK_COMMAND_HEADER + PK_CMD_HI], cmd_id);
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_0] = cmd1;
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_1] = cmd2; 
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_2] = cmd3; 
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_3] = cmd4;
}

void set_request_id(uint8_t* buff)
{ 
	
    //formerly: word_to_bytes(& buff[PK_COMMAND_HEADER + PK_CMD_DATA_0], req_id);
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_0] = req_id;
    req_id++;   
}

void send_command(uint16_t target, uint8_t* buff)
{
    mrf_send16(target,buff,buff[PK_COMMAND_HEADER + PK_SZ_PACKET]);
}

//specific functions for pinging and initializing setup routine
void send_ping(uint16_t target, uint8_t* buff)
{	
	set_packet_size(buff,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    	set_target_node(buff,target);
	set_command(buff,CMD_PING,0,1,2,3);
 	set_request_id(buff);
	send_command(target, buff);
}

void send_setup(uint16_t target, uint8_t* buff)
{
	set_packet_size(buff,PK_SZ_TXRX_BUFFER); //packet will always have all 40
    	set_target_node(buff,target);
	set_command(buff,CMD_SETUP,0,0,0,0);
 	set_request_id(buff);
	send_command(target, buff);
	memset(buff,0,sizeof(buff));
}

//For getting data from the message
uint16_t get_source_node_address(uint8_t* msg)
{
    return bytes_to_word(& msg[PK_SRC_ADDR_HI]);
}

uint8_t get_message_size(uint8_t* msg)
{
	// just a 1 byte
	return msg[PK_COMMAND_HEADER + PK_SZ_PACKET];
}

uint16_t get_command(uint8_t* msg)
{	
    return bytes_to_word(& msg[PK_COMMAND_HEADER+PK_CMD_HI]);
	
}

uint8_t get_command_data(uint8_t* msg, uint8_t n)
{
    //return bytes_to_word(& msg[PK_COMMAND_HEADER + PK_CMD_DATA_0 + n]);
    return msg[PK_COMMAND_HEADER + PK_CMD_DATA_0 + n];
}

uint16_t get_target_node(uint8_t* msg)
{ 
    return bytes_to_word(& msg[PK_DEST_ADDR_HI]);
}


void print_message(uint8_t * msg)
{
    printf("\n\n===============================================");
    printf("\nMessage from node:0x%x",get_source_node_address(msg));
    printf("\nMessage to node:0x%x",get_target_node(msg));
    printf("\nCommand:0x%x ",get_command(msg));
    printf("\nCommand data: 0x%x 0x%x 0x%x 0x%x",get_command_data(msg,0),get_command_data(msg,1),get_command_data(msg,2),get_command_data(msg,3));
    printf("\nMessage size: %d",get_message_size(msg));
    //printf("\nSensor data size: %d",get_data_size(msg));
    //printf("\nHop count: %d",get_hop_count(msg));
    printf("\n===============================================\n");
}

void main() //just send the setup routine
{
	setup();
	printf("\nSetup Complete"); fflush(stdout);
	send_setup(0x0001,transmit_data_buffer); fflush(stdout);
	printf("\nMessage Sent\n"); fflush(stdout);
	print_message(transmit_data_buffer);
	//printf("\n%x",bytes_to_word(&transmit_data_buffer[10]));
	fflush(stdout);
	while(1)
	{
		mrf_check_flags(&handle_rx, &handle_tx);
		if (get_message_status() == 1) {print_received_message();  set_message_status(0);}
	}
	
	
}