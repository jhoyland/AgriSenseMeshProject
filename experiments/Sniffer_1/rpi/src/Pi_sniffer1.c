/* This program will simply send out requests, and listen for data being exchanged by the nodes
It does this by looking for packets on the same PANID (0xCAFE) */

#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiSPI.h> /*JAMES: Added this - was causing implicit declaration warning*/
#include <string.h>
#include "simple_queue.h"
#include "mrf24jpi.h"
#include "pktspec.h"
#include "netspec.h"
#include "cmdspec.h"
#include "bitmanip.h"/*JAMES: Added this - was causing implicit declaration warning*/

//which GPIO pin we're using
#define INT_PIN 0 //the interrupt pin

uint16_t req_id;
uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t error_data_buffer[PK_SZ_ERR_BUFFER];
uint8_t* transmit_command_header;
uint8_t active_message[PK_SZ_TXRX_BUFFER];




void handle_rx() {

    memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()*sizeof(uint8_t)); // Copy the message into the recieved data buffer

    char x[4];

    x[0] = recieved_data_buffer[0];
    x[1] = recieved_data_buffer[1];
    x[2] = recieved_data_buffer[2];
    x[3] = recieved_data_buffer[3];

    printf("\nGot: %s", recieved_data_buffer);

   // if(! enqueue( &message_queue, recieved_data_buffer ))                               // Try to queue the received message
   // {
   //     printf("\nCommand queue full. Lost node message");
   // }
   // else
   // {
  //      printf("\nNode message queued");
   // }

}

// This is called by the interrupt handler when transmit has completed and the receiver has acknowledged
// The acknowledgement is automatic however, note it is the actual receiver of this message which sends
// acknowledge - not the final destination node. In this case all it tells us is if the BASE_NODE of the 
// network has received the message

void handle_tx() {

 //   tx_info_t * inf;

   // mrf_get_txinfo(&inf);

  //  printf("\nStatus register:%x  &0x3F = %x     ! = %d",mrf_reg_TXSTAT,mrf_reg_TXSTAT & 0x3F,!(mrf_reg_TXSTAT & 0x3F));
	
    if (!(mrf_reg_TXSTAT & 0x3F)/*mrf_tx_ok()*/) {
    //    printf("\nTX ACK");
    } else {
      //  printf("\nTX FAIL");
    }
}

void set_packet_header(uint8_t* buff)
{

    buff[PK_DEST_PANID_HI] = PAN_ID_HI;
    buff[PK_DEST_PANID_LO] = PAN_ID_LO;
    buff[PK_SRC_PANID_HI] = PAN_ID_HI;
    buff[PK_SRC_PANID_LO] = PAN_ID_LO;
    buff[PK_SRC_ADDR_HI] = PI_ADDR_HI;
    buff[PK_SRC_ADDR_LO] = PI_ADDR_LO;
    buff[PK_COMMAND_HEADER+PK_HOP_COUNT] = 0;

}

void setup()
{
 	//setup_queue(&message_queue,SZ_MESSAGE_QUEUE,SZ_MESSAGE,0);
    	transmit_command_header = &transmit_data_buffer[PK_COMMAND_HEADER];


    	wiringPiSetup();
    	wiringPiSPISetup (0, 1000000) ;

	// Set pin to output in case it's not
    	pinMode(INT_PIN, OUTPUT);
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
  	 // req_id = 1;

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

void set_command(uint8_t* buff, uint16_t cmd_id, uint8_t cmd2, uint8_t cmd3)
{
    word_to_bytes(& buff[PK_COMMAND_HEADER + PK_CMD_HI], cmd_id);
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_2] = cmd2; 
    buff[PK_COMMAND_HEADER + PK_CMD_DATA_3] = cmd3; 
}

void set_request_id(uint8_t* buff)
{ 
    word_to_bytes(& buff[PK_COMMAND_HEADER + PK_CMD_DATA_0], req_id);
    req_id++;   
}

void send_command(uint16_t target, uint8_t* buff)
{
    mrf_send16(target,buff,buff[PK_COMMAND_HEADER + PK_SZ_PACKET]);
}


void send_ping(uint16_t target, uint8_t* buff)
{	
	set_packet_size(buff,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    	set_target_node(buff,target);
      /*JAMES: Added 1,2 to produce test out put in CMD_DATA*/
	set_command(buff,CMD_PING,1,2);
 	set_request_id(buff);
	send_command(target, buff);
}

//For getting data from the message
uint16_t get_source_node_address(uint8_t* msg)
{
    return bytes_to_word(& msg[PK_SRC_ADDR_HI]);
}
 
uint8_t get_message_size(uint8_t* msg)
{
  /*
    JAMES: You don't need to "bytes_to_word" the message size, it's only a single 
    byte, you can just return it


    return bytes_to_word(& msg[PK_COMMAND_HEADER + PK_SZ_PACKET]);*/

  return msg[PK_COMMAND_HEADER + PK_SZ_PACKET];
}

uint16_t get_command(uint8_t* msg)
{	//msg[10] = 5;	//if I do this, it seems to make element 10 equal to 5.
    return bytes_to_word(& msg[PK_COMMAND_HEADER+PK_CMD_HI]);
	//return bytes_to_word(& msg[10]);
}

uint8_t get_command_data(uint8_t* msg, uint8_t n)
{
    return bytes_to_word(& msg[PK_COMMAND_HEADER + PK_CMD_DATA_0 + n]);
}

void print_message(uint8_t * msg)
{
    printf("\n\n===============================================");
    printf("\nMessage from node:0x%x",get_source_node_address(msg));
    printf("\nCommand:0x%x ",get_command(msg));
    printf("\nCommand data: 0x%x 0x%x 0x%x 0x%x",get_command_data(msg,0),get_command_data(msg,1),get_command_data(msg,2),get_command_data(msg,3));
    printf("\nMessage size: %d",get_message_size(msg));
    //printf("\nSensor data size: %d",get_data_size(msg));
    //printf("\nHop count: %d",get_hop_count(msg));
    printf("\n===============================================\n");
}

void main()
{
	setup();
	printf("\nSetup Complete");
	send_ping(0x0001,transmit_data_buffer);  /*JAMES:uncommented*/
	printf("\nMessage Sent\n");
	print_message(transmit_data_buffer);
  /* JAMES: Again this will combine bytes 9 and 10 of the buffer into a single 16-bit value */
	/*printf("\n%x",bytes_to_word(&transmit_data_buffer[9]));*/
  printf("\n%x",transmit_data_buffer[9]);
	fflush(stdout);
	while(1)
	{
		mrf_check_flags(&handle_rx, &handle_tx);
	}
	
	
}