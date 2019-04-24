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


// Time of last change
struct timeval last_change;

uint16_t req_id;
volatile uint8_t keep_going;

uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER];
uint8_t error_data_buffer[PK_SZ_ERR_BUFFER];
uint8_t* transmit_command_header;
uint8_t active_message[PK_SZ_TXRX_BUFFER];


#define SZ_MESSAGE_QUEUE 8
#define SZ_MESSAGE PK_SZ_TXRX_BUFFER

struct simple_queue message_queue;

/* Functions to set up outgoing messages */

void set_packet_header()
{
    transmit_data_buffer[PK_DEST_PANID_HI] = PAN_ID_HI;
    transmit_data_buffer[PK_DEST_PANID_LO] = PAN_ID_LO;
    transmit_data_buffer[PK_SRC_PANID_HI] = PAN_ID_HI;
    transmit_data_buffer[PK_SRC_PANID_LO] = PAN_ID_LO;
    transmit_data_buffer[PK_SRC_ADDR_HI] = PI_ADDR_HI;
    transmit_data_buffer[PK_SRC_ADDR_LO] = PI_ADDR_LO;
    transmit_data_buffer[PK_COMMAND_HEADER+PK_HOP_COUNT] = 0;
}

void set_request_id()
{
    transmit_command_header[PK_CMD_DATA_0] = (uint8_t)(req_id>>8);  // This is an reference number for the request - should be unique to each request
    transmit_command_header[PK_CMD_DATA_1] = (uint8_t)(255&req_id); 
    req_id++;   
}

void set_target_node(uint16_t target_node)
{
    transmit_data_buffer[PK_DEST_ADDR_HI] = (uint8_t)(target_node>>8);
    transmit_data_buffer[PK_DEST_ADDR_LO] = (uint8_t)(255&target_node);    
}

void set_command(uint16_t cmd_id, uint8_t cmd2, uint8_t cmd3)
{
    transmit_command_header[PK_CMD_HI] = (uint8_t)(cmd_id>>8);   
    transmit_command_header[PK_CMD_LO] = (uint8_t)(255&cmd_id);
    transmit_command_header[PK_CMD_DATA_2] = cmd2; 
    transmit_command_header[PK_CMD_DATA_3] = cmd3; 
}

void set_packet_size(uint8_t sz)
{

    transmit_command_header[PK_SZ_PACKET] = sz;
}

void send_command()
{
    mrf_send16(GATEWAY_NODE,transmit_data_buffer,transmit_command_header[PK_SZ_PACKET]);
}

/* Convenience functions for extracting data from message*/

uint16_t get_command(uint8_t* msg)
{
    return bytes_to_word(& msg[PK_COMMAND_HEADER+PK_CMD_HI]);
}

uint8_t get_command_data(uint8_t* msg, uint8_t n)
{
    return msg[PK_COMMAND_HEADER + PK_CMD_DATA_0 + n];
}

uint16_t get_source_node_address(uint8_t* msg)
{
    return bytes_to_word(& msg[PK_SRC_ADDR_HI]);
}

uint8_t get_message_size(uint8_t* msg)
{
    return msg[PK_COMMAND_HEADER + PK_SZ_PACKET];
}

uint8_t get_data_size(uint8_t* msg)
{

    if(get_command(msg) != CMD_DATA) return 0;

    uint8_t datsz = get_message_size(msg) - PK_SZ_ADDR_HEADER - PK_SZ_CMD_HEADER;

    if(datsz == 0) return 0;
    if(datsz % PK_DATUM_SIZE != 0) return 0;

    return datsz / PK_DATUM_SIZE;
}

void get_data_array(uint8_t* msg, uint8_t n, uint16_t * out)
{
    unpack_12bit(out,& msg[PK_DATA_START + n * PK_DATUM_SIZE]);
}

void get_data(uint8_t *msg, uint8_t n, uint16_t* d1, uint16_t* d2)
{
    uint16_t dat[2];
    get_data_array(msg,n,dat);
    *d1 = dat[0];
    *d2 = dat[1];
}

uint8_t get_hop_count(uint8_t* msg)
{
    return msg[PK_COMMAND_HEADER + PK_HOP_COUNT];
}

/* Form requests for specific commands */


void request_data(uint16_t target_node)
{
    set_packet_size(PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(target_node);
    set_command(CMD_DATA,1,0);
    set_request_id();
    send_command();
}

void request_ping(uint16_t target_node, uint8_t d)
{
    set_packet_size(PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(target_node);
    set_command(CMD_PING,d,0);
    set_request_id();
    send_command();
}

/* Process the next message in the queue */

void process_next_node_message()
{
    if(!dequeue(&message_queue,active_message))
    {
        return;
    }

    uint16_t cmd = bytes_to_word(& active_message[PK_COMMAND_HEADER + PK_CMD_HI]);

    switch(cmd)
    {
        case CMD_DATA:
            printf("\nProcessing data message");
            process_node_data();
            break;
        case CMD_ECHO:
            printf("\nProcessing ping echo");
            process_node_ping_echo();
            break;
        default:
            printf("\n*Unrecognized command*");
            print_message(active_message);
    }


}

void process_node_data()
{
    print_message(active_message);
    print_data(active_message);
}

void process_node_ping_echo()
{
    print_message(active_message);
}

void print_message(uint8_t * msg)
{
    printf("\n\n===============================================");
    printf("\nMessage from node:0x%x",get_source_node_address(msg));
    printf("\nCommand:0x%x ",get_command(msg));
    printf("\nCommand data: 0x%x 0x%x 0x%x 0x%x",get_command_data(msg,0),get_command_data(msg,1),get_command_data(msg,2),get_command_data(msg,3));
    printf("\nMessage size: %d",get_message_size(msg));
    printf("\nSensor data size: %d",get_data_size(msg));
    printf("\nHop count: %d",get_hop_count(msg));
    printf("\n===============================================\n");
}

void print_data(uint8_t * msg)
{

    uint8_t datsz = get_data_size(msg);

    uint16_t dat_mean;
    uint16_t dat_stderr;

    if(datsz > 0)
    {
        uint8_t n = 0;
        uint8_t m = 0;
        uint8_t channel_read_ok = get_command_data(msg,2);
        uint8_t channel_requested = get_command_data(msg,3);

        printf("\nSENSOR DATA:\n============\n");

        for(n = 0; n < ADC_N_CHANNELS; n++)
        {
            printf("\nCh %d:",n);
            if(channel_requested & (1 << n))
            {
                if(channel_read_ok & (1 << n))
                {
                    get_data(msg,n,&dat_mean,&dat_stderr);
                    printf(" &d \t +/- \t &d", dat_mean, dat_stderr);

                }
                else
                {
                    printf(" READ FAILED");
                }
            }
            else
            {
                printf(" NOT REQUESTED");
            }
        }
    }
    else
    {
        printf("\nNO DATA");
    }
}

// This is called by the interrupt handler if new data is received

void handle_rx() {

    memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()*sizeof(uint8_t)); // Copy the message into the recieved data buffer
    if(! enqueue( &message_queue, recieved_data_buffer ))                               // Try to queue the received message
    {
        printf("\nCommand queue full. Lost node message");
    }
    else
    {
        printf("\nNode message queued");
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

    setup_queue(&message_queue,SZ_MESSAGE_QUEUE,SZ_MESSAGE,0);

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

    set_packet_header();

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
        request_ping(GATEWAY_NODE,0x42);
        last_change = new_time; 
        keep_going = keep_going - 1;
    } 

    process_next_node_message();

                     
}

int main(void)
{
    setup();
    while(keep_going) loop();
    return 0;   
}
