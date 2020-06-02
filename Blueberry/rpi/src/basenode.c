#include <stdio.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <string.h>
#include "simple_queue.h"
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

void set_request_id(uint8_t* buff)
{ 
    word_to_bytes(& buff[PK_COMMAND_HEADER + PK_CMD_DATA_0], req_id);
    req_id++;   
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

void set_packet_size(uint8_t* buff, uint8_t sz)
{

    buff[PK_COMMAND_HEADER + PK_SZ_PACKET] = sz;
}

void send_command(uint8_t* buff)
{
    mrf_send16(GATEWAY_NODE,buff,buff[PK_COMMAND_HEADER + PK_SZ_PACKET]);
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
    set_packet_size(transmit_data_buffer,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(transmit_data_buffer,target_node);
    set_command(transmit_data_buffer,CMD_DATA,3,0);
    set_request_id(transmit_data_buffer);
    send_command(transmit_data_buffer);
}

void request_ping(uint16_t target_node, uint8_t d)
{
    set_packet_size(transmit_data_buffer,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(transmit_data_buffer,target_node);
    set_command(transmit_data_buffer,CMD_PING,d,0);
    set_request_id(transmit_data_buffer);
    send_command(transmit_data_buffer);
}

void request_set_param(uint16_t target_node, uint8_t c)
{
    set_packet_size(transmit_data_buffer,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(transmit_data_buffer,target_node);
    set_command(transmit_data_buffer,CMD_SET_PARAMETER,c,0);
    set_request_id(transmit_data_buffer);
    send_command(transmit_data_buffer);    
}

void request_get_param(uint16_t target_node)
{
    set_packet_size(transmit_data_buffer,PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER);
    set_target_node(transmit_data_buffer,target_node);
    set_command(transmit_data_buffer,CMD_GET_PARAMETER,0,0);
    set_request_id(transmit_data_buffer);
    send_command(transmit_data_buffer);    
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
<<<<<<< HEAD
   //         printf("\nProcessing data message");
=======
           // printf("\nProcessing data message");
>>>>>>> 762b8d47354e5494bf4020b744a895b1a8843c98
            process_node_data();
            break;
        case CMD_ECHO:
           // printf("\nProcessing ping echo");
            process_node_ping_echo();
            break;
        case CMD_NODE_TEST:
            printf("\nNode Test");
            print_message(active_message);
            break;
        default:
            printf("\n*Unrecognized command*");
            print_message(active_message);
    }


}

void process_node_data()
{
<<<<<<< HEAD
    //print_message(active_message);
=======
//print_message(active_message);
>>>>>>> 762b8d47354e5494bf4020b744a895b1a8843c98
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

<<<<<<< HEAD
    //    printf("\nSENSOR DATA:\n============\n");
=======
        printf("\nSENSOR DATA: ");
>>>>>>> 762b8d47354e5494bf4020b744a895b1a8843c98

        for(n = 0; n < ADC_N_CHANNELS; n++)
        {
           // printf("\nCh %d:",n);
            if(channel_requested & (1 << n))
            {
                if(channel_read_ok & (1 << n))
                {
                    get_data(msg,n,&dat_mean,&dat_stderr);
                    printf("%d: %4.0d +/- %4.0d, ",n, dat_mean, dat_stderr);

                }
                else
                {
                    printf("%d: F, ",n);
                }
            }
            else
            {
                printf("%d: N, ",n);
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

    /*char x[4];

    x[0] = recieved_data_buffer[0];
    x[1] = recieved_data_buffer[1];
    x[2] = recieved_data_buffer[2];
    x[3] = recieved_data_buffer[3];

    printf("\nGot: %s", recieved_data_buffer);*/

    if(! enqueue( &message_queue, recieved_data_buffer ))                               // Try to queue the received message
    {
        printf("\nCommand queue full. Lost node message");
    }
    else
    {
  //      printf("\nNode message queued");
    }

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

void setup() {

    setup_queue(&message_queue,SZ_MESSAGE_QUEUE,SZ_MESSAGE,0);
    transmit_command_header = &(transmit_data_buffer[PK_COMMAND_HEADER]);


    wiringPiSetup();
    wiringPiSPISetup (0, 1000000) ;

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

    set_packet_header(transmit_data_buffer);

}

uint8_t count;
uint8_t toggle;

void loop() {
	
	// Check if any new interrupts have triggered
    mrf_check_flags(&handle_rx, &handle_tx);

    struct timeval new_time;
    gettimeofday(&new_time,NULL);

	// request data every 5 seconds
	
    if( (new_time.tv_sec - last_change.tv_sec) > 5 )
    {

     /*   printf("\nWaiting %i\n", req_id);
        if(toggle) 
            {
                request_set_param(GATEWAY_NODE,count);
                toggle = 0;
            }
            else
            {
                request_get_param(GATEWAY_NODE);
                toggle = 1;
            }*/

        request_data(GATEWAY_NODE);

        last_change = new_time; 
        keep_going = keep_going - 1;
        count++;
    } 

    process_next_node_message();

                     
}

int main(void)
{
    count = 0;
    toggle = 0;
    setup();
    while(keep_going) loop();
    return 0;   
}
