
//#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "sensenode.h"
#include "blinkin.h"

#include <math.h> 

#define SZ_COMMAND_BUFFER 5
#define SZ_COMMAND PK_SZ_CMD_HEADER


uint8_t transmit_data_buffer[PK_SZ_TXRX_BUFFER]; // Buffer for transmitted message
uint8_t recieved_data_buffer[PK_SZ_TXRX_BUFFER]; // Buffer for received message
uint8_t queue_memory[SZ_COMMAND * SZ_COMMAND_BUFFER + 1];  // Command queue buffer
uint8_t test_command[SZ_COMMAND];  // test command used for debugging
uint8_t error_data_buffer[PK_SZ_ERR_BUFFER];  //  Buffer for transmitting error message
uint8_t* transmit_command_header;   // Convenience pointer to the command portion of the transmit buffer
uint8_t adc_buffer[3];  // Buffer for transfer for control of the ADC
uint8_t* active_command;  // Pointer to the current active command - physical memory for this lies in the "data" section of the error buffer so it is 
							// automatically transmitted as part of any error message

// Status flags are set and unset during operation and packaged with error messages

uint8_t startup_status = 0;  // Status flags for during the power-on setup sequence
uint8_t running_status = 0;  // Status flags for normal operation

// Queue of commands. The queue holds up to SZ_COMMAND_BUFFER commands

struct simple_queue command_queue;

// Currently active ADC channels

uint8_t adc_active_channels_bitmask = 3;

// This node's address

uint16_t my_address;

#define __DIAGNOSTIC_LEDS__

// Diagnostic LEDS for testing

#ifdef __DIAGNOSTIC_LEDS__

#define __FLASH_RED__ BLINK(LED_PORT,LED_1)
#define __FLASH_YELLOW__ BLINK(LED_PORT,LED_2)
#define __FLASH_GREEN__ BLINK(LED_PORT,LED_3)

#define __RED_ON__ LED_ON(LED_PORT,LED_1)
#define __YELLOW_ON__ LED_ON(LED_PORT,LED_2)
#define __GREEN_ON__ LED_ON(LED_PORT,LED_3)

#define __RED_OFF__ LED_OFF(LED_PORT,LED_1)
#define __YELLOW_OFF__ LED_OFF(LED_PORT,LED_2)
#define __GREEN_OFF__ LED_OFF(LED_PORT,LED_3)

#else

#define __FLASH_RED__
#define __FLASH_YELLOW__
#define __FLASH_GREEN__

#define __RED_ON__
#define __YELLOW_ON__
#define __GREEN_ON__ 

#define __RED_OFF__
#define __YELLOW_OFF__
#define __GREEN_OFF__ 

#endif

uint16_t big_counter = 0;

// Recieve handler - called by check flags function if a the MRF reports a message has been received

void handle_rx()
{
	running_status |= (1<<RU_RX_HANDLE);

	// Copy the received data from the mrf to the received data buffer

	memcpy(recieved_data_buffer,mrf_get_rxdata(),mrf_rx_datalength()*sizeof(uint8_t));

	// Check of the recieved packet is addressed to this node

	if( bytes_to_word(& recieved_data_buffer[PK_DEST_ADDR_HI]) == my_address )
	{
		// Queue the received command. Only the command header section of the message is stored in the queue

		enqueue( &command_queue, &recieved_data_buffer[PK_COMMAND_HEADER] ); 
		// TO DO: Check that enqueue worked. Currently the command is lost if queue is full - should send error to base in this case
	}
	else
	{
		// TO DO: Send the received data on to the next node

		send_error(bytes_to_word(& recieved_data_buffer[PK_DEST_ADDR_HI]));
		/*
		recieved_data_buffer[PK_COMMAND_HEADER + PK_HOP_COUNT] ++;

		if(recieved_data_buffer[PK_DEST_ADDR_HI] == PI_ADDR_HI && recieved_data_buffer[PK_DEST_ADDR_LO] == PI_ADDR_LO)

			send_downstream(recieved_data_buffer);

		else

			send_upstream(recieved_data_buffer);*/

	}

	running_status &= ~(1<<RU_RX_HANDLE);
}

// Handles the notification of the transmission of data - doesn't do much yet

void handle_tx()
{
	running_status |= (1<<RU_TX_HANDLE);

	//__YELLOW_ON__;

	if(mrf_tx_ok())
	{
//		__FLASH_GREEN__;
	}
	else
	{
//		__FLASH_RED__;
	}

	//__YELLOW_OFF__;

	running_status &= ~(1<<RU_TX_HANDLE);

}

// Read a single value from the ADC chip. This is a 3-byte SPI transfer. Returns the value as a 16 bit unsigned integer

uint16_t get_adc_value(uint8_t chan)
{
	adc_buffer[0] = 6 | (chan >>2);
	adc_buffer[1] = chan << 6;
	
	spi_transfer_nbytes(adc_buffer,adc_buffer,3,ADC_CS);
	
	uint16_t b1 = adc_buffer[1];
	uint16_t b2 = adc_buffer[2];
	
	return b2 | (b1<<8);
}

// Read NP_ADC_N_SAMPLES and find their average and standard deviation. The mean and sd are both 12-bit values which are returned packed
// into 3 8-bit integers -  problems getting this to work
 
uint8_t get_packed_data(uint8_t * op, uint8_t ch)
{
	float d = 0; 		// Store individual data values
	float mean = 0;
	float serr = 0;
	uint16_t data[2];
	data[0] = data[1] = 0;

	for(uint16_t i=0; i<NP_ADC_N_SAMPLES; i++)
	{
		d = (float) get_adc_value(ch);		// Get the data point
		mean  += d;		// Accumulate data point
		serr  += d*d;		// Square and accumulate data point
	}

	mean = mean / NP_ADC_N_SAMPLES;	// Calculate mean
	serr = sqrt( (serr - NP_ADC_N_SAMPLES*mean*mean)/(NP_ADC_N_SAMPLES*(NP_ADC_N_SAMPLES-1.0)) );  // Calculate standard error

	mean = fmin(mean,0xFFF);
	serr = fmax(serr,1);			// Minimum 1 bit error

	data[0] = mean;
	data[1] = serr;

	if(data[1]==0) data[1] = 1;

	pack_12bit(op,data); // Pack the data into the supplied array

	return 1;
}

// Command function for get data request
// 

void command_get_data(uint8_t* command)
{
	running_status |= (1<<RU_DATA_COLLECT);

	// Set the command code in the tx buffer to "DA"
	word_to_bytes(&transmit_command_header[PK_CMD_HI],CMD_DATA);

	// Copy the request id from the command into the tx buffer as command data
	transmit_command_header[PK_CMD_DATA_0] = command[PK_CMD_DATA_0]; // REQUEST ID HI
	transmit_command_header[PK_CMD_DATA_1] = command[PK_CMD_DATA_1]; // REQUEST ID LO

	// Set the requested ADC channels
	uint8_t adc_channel_request_bitmask = command[PK_CMD_DATA_2]; // Requested ADC channels
	uint8_t adc_read_ok_bitmask = 0; // Holds the success status of each channel
	transmit_command_header[PK_SZ_PACKET] = PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER;  // Initial transmit packet size is just address and command header lengths
	uint8_t* data_pointer = &transmit_data_buffer[PK_DATA_START];  // Sensor data will be read directly into the transmit buffer data section


	// Scan through the ADC channels
	for(uint8_t i=0;i<ADC_N_CHANNELS;i++)
	{
		// ADC read performed if channel 'i' is requested and active
		if((adc_channel_request_bitmask & adc_active_channels_bitmask) & (1<<i))
		{
			if(get_packed_data(data_pointer, i)) 
			{
				// ADC read is succesful update read status flag and packet size, move data pointer to next slot
				adc_read_ok_bitmask |= (1<<i);
				transmit_command_header[PK_SZ_PACKET] += 3;
				data_pointer += 3;
			}
		}
	}



	running_status &= ~(1<<RU_DATA_COLLECT);

	transmit_command_header[PK_CMD_DATA_2] = adc_read_ok_bitmask;
	transmit_command_header[PK_CMD_DATA_3] = adc_active_channels_bitmask;

	send_downstream(transmit_data_buffer);
}

void command_ping(uint8_t* command)
{
	word_to_bytes(&transmit_command_header[PK_CMD_HI],CMD_ECHO);
	transmit_command_header[PK_SZ_PACKET] = PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER;
	transmit_command_header[PK_CMD_DATA_0] = command[PK_CMD_DATA_0];
	transmit_command_header[PK_CMD_DATA_1] = command[PK_CMD_DATA_1];
	transmit_command_header[PK_CMD_DATA_2] = command[PK_HOP_COUNT];
	transmit_command_header[PK_CMD_DATA_3] = 0x50;

	send_downstream(transmit_data_buffer);
}


void set_downstream_address_header(uint8_t* buff)
{
	buff[ PK_DEST_PANID_HI ] = PAN_ID_HI;
	buff[ PK_DEST_PANID_LO ] = PAN_ID_LO;
	buff[ PK_DEST_ADDR_HI ]   = PI_ADDR_HI;
	buff[ PK_DEST_ADDR_LO ]   = PI_ADDR_LO;
	buff[ PK_SRC_PANID_HI ] = PAN_ID_HI;
	buff[ PK_SRC_PANID_LO ] = PAN_ID_LO;

	word_to_bytes(&buff[PK_SRC_ADDR_HI],my_address);
}

/* Get / set parameter commands will eventially allow modification of important parameters accross the network - right now they just turn the
LEDs on and off for diagnostic purposes */

void command_set_parameter(uint8_t* cmd)
{
	uint8_t leds = cmd[PK_CMD_DATA_2];

	if(leds & 1) {__GREEN_ON__;}
		else {__GREEN_OFF__;}

	if(leds & 2) {__YELLOW_ON__;}
		else {__YELLOW_OFF__;}

	if(leds & 4) {__RED_ON__;}
		else {__RED_OFF__;}
}

void command_get_parameter(uint8_t* cmd)
{
	uint8_t led_state = 0;

	if(LED_PORT & (1 << LED_3)) led_state = 1;
	if(LED_PORT & (1 << LED_2)) led_state |= 2;
	if(LED_PORT & (1 << LED_1)) led_state |= 4;

	word_to_bytes(&transmit_command_header[PK_CMD_HI],CMD_GET_PARAMETER);
	transmit_command_header[PK_SZ_PACKET] = PK_SZ_ADDR_HEADER + PK_SZ_CMD_HEADER;
	transmit_command_header[PK_CMD_DATA_0] = cmd[PK_CMD_DATA_0];
	transmit_command_header[PK_CMD_DATA_1] = cmd[PK_CMD_DATA_1];
	transmit_command_header[PK_CMD_DATA_2] = led_state;

	set_downstream_address_header(transmit_data_buffer);

	send_downstream(transmit_data_buffer);

}

void command_send_test()
{
//	__FLASH_RED__;
	memcpy(transmit_command_header,test_command,SZ_COMMAND);
	set_downstream_address_header(transmit_data_buffer);

	send_downstream(transmit_data_buffer);
}

void execute_next_command()
{
	if(isr_lock) return;
	if(dequeue(&command_queue,active_command) == 0) 
	{
		return; // No commands waiting
	}
	running_status |= (1<<RU_CMD_EXEC);

	uint16_t command = bytes_to_word(& active_command[PK_CMD_HI]);
	switch (command) {
		case CMD_DATA:
			command_get_data(active_command);
			break;
		case CMD_PING:
			command_ping(active_command);
			break;
	/*	case CMD_QSTATUS:
			command_status_report(active_command);
			break;*/
		case CMD_SET_PARAMETER:
			command_set_parameter(active_command);
			break;
		case CMD_GET_PARAMETER:
			command_get_parameter(active_command);
			break;
		case CMD_NODE_TEST:
			command_send_test();
			break;
		default:
			send_error(ERR_UNRECOGNIZED_COMMAND);
	}
	running_status &= ~(1<<RU_CMD_EXEC);

	memset(active_command,0,PK_SZ_CMD_HEADER*sizeof(uint8_t)); // zeroing so that errors sent outside of command execution do not contain old commands
}

// Rigid grid address system - 

void send_downstream(uint8_t* msg)
{
	uint16_t next_downstream_node = 0;

	if(my_address == 0x1010) // I am the gateway node
	{
		next_downstream_node = PI_ADDR;
	}
	else
	{
		next_downstream_node = my_address;

		if( (my_address & 0xFF) == 0x10 ) // Row number is 0x10 so I am the top of a column

			next_downstream_node = next_downstream_node - 0x0100;

		else

			next_downstream_node = next_downstream_node - 0x01; // reduces row number

	}

	mrf_send16(next_downstream_node,msg,msg[PK_COMMAND_HEADER + PK_SZ_PACKET]);
}


void send_upstream(uint8_t* msg)
{
	uint16_t next_upstream_node = my_address;


	if(msg[PK_DEST_ADDR_HI] == (my_address & 0xFF00))  // The destination is in my column
	{
		next_upstream_node += 0x01; // Increase row number
	}
	else
	{
		next_upstream_node += 0x0100;  // Increase column number
	}
	
	mrf_send16(next_upstream_node,msg,msg[PK_COMMAND_HEADER + PK_SZ_PACKET]);

}


void send_error(uint16_t xm)
{
	set_downstream_address_header(error_data_buffer);

	error_data_buffer[PK_COMMAND_HEADER + PK_HOP_COUNT] = 0;
	error_data_buffer[PK_COMMAND_HEADER + PK_SZ_PACKET] = PK_SZ_ADDR_HEADER + 2 * PK_SZ_CMD_HEADER; // active_command pointer already  in error buffer

	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_HI] = 0x58;	//'XX'
	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_LO] = 0x58;

	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_DATA_0] = (xm >> 8);
	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_DATA_1] = (xm & 0xFF);
	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_DATA_2] = startup_status;
	error_data_buffer[PK_COMMAND_HEADER + PK_CMD_DATA_3] = running_status;

	send_downstream(error_data_buffer);

}

void setup_ports()
{

  DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK); 
  DDRB |= (1<<ADC_CS) | (1<<MRF_CS);  

  DDRD |= (1<<LED_1) | (1<<LED_2) | (1<<LED_3);  
  DDRD |= (1<<MRF_WAKE) | (1<<MRF_RESET);

  PORTB |= (1<<MRF_CS); //
  PORTB |= (1<<ADC_CS);

  MRF_RESET_PORT |= (1<<MRF_INT);
  MRF_RESET_PORT |= (1<<MRF_RESET);
}

void setup() {

  uint8_t ok = 0;

  startup_status = 0b11111111;

  running_status = (1<<RU_SETUP);

  /* Data directions */
  setup_ports();
 

  startup_status &= ~(1<<ST_PORTS);
  __FLASH_GREEN__;

  spi_set_data_direction(SPI_MSB);
  spi_setup();
  spi_set_speed(SCK_F_64); // MCP3208 doesn't like SPI speed too high. This sets it to fosc / 64



  startup_status &= ~(1<<ST_SPI);
  __FLASH_GREEN__;

  _delay_ms(500);

  mrf_reset();  
  mrf_init();
  mrf_set_pan(PAN_ID);
  my_address = MY_ADDRESS;
  mrf_address16_write(my_address); 

  startup_status &= ~(1<<ST_MRF);
  __FLASH_GREEN__;

  ok = setup_queue_static(&command_queue,queue_memory,SZ_COMMAND_BUFFER,SZ_COMMAND,0); // Dynamic memory allocation causes too many problems!

  transmit_command_header = & transmit_data_buffer[PK_COMMAND_HEADER];
  set_downstream_address_header(transmit_data_buffer);
  set_downstream_address_header(error_data_buffer);
  active_command = & error_data_buffer[PK_DATA_START]; // Active command is located in the error data buffer so it is automatically sent with errors 

  if(ok) 
  	{
  		startup_status &= ~(1<<ST_BUFFERS);
  		__FLASH_GREEN__;
  	}
  	else
  	{
  		__RED_ON__;
  	}

  ok = 0;

  EIMSK |= (1<<INT0);
  EICRA |= (1<<ISC01);
  sei();

  startup_status &= ~(1<<ST_INTERRUPTS);
  __FLASH_GREEN__;
  startup_status &= ~(1<<ST_POWER);			// TO DO: Power check
  __FLASH_YELLOW__;
  startup_status &= ~(1<<ST_SENSORS);		// TO DO: Power check
  __FLASH_YELLOW__;
  startup_status &= ~(1<<ST_OPTION);
  __FLASH_YELLOW__;


  test_command[PK_CMD_HI] = 0x4A;
  test_command[PK_CMD_LO] = 0x4A;
  test_command[PK_HOP_COUNT] = 0;
  test_command[PK_SZ_PACKET] = PK_SZ_CMD_HEADER + PK_SZ_ADDR_HEADER;
  test_command[PK_CMD_DATA_0] = 0x41;
  test_command[PK_CMD_DATA_1] = 0x42;
  test_command[PK_CMD_DATA_2] = 0x43;
  test_command[PK_CMD_DATA_3] = 0x44;

  running_status &= ~(1<<RU_SETUP);
}

ISR(INT0_vect) {
	running_status |= (1<<RU_INTERRUPT);
    mrf_interrupt_handler(); // mrf24 object interrupt routine
    running_status &= ~(1<<RU_INTERRUPT);
}

void loop() {
    mrf_check_flags(&handle_rx, &handle_tx);

    if(startup_status == 0) execute_next_command();
			
}

int main(void)
{
    setup();
    running_status |= (1<<RU_RUNNING);
    while(1) loop();

    return 0;
}



