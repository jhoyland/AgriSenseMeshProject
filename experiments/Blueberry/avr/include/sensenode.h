#ifndef __SENSENODE_H
#define __SENSENODE_H
/*
 * sensenode-Blueberry-0.0.1
 *
 * Created: 2019-02-04 7:27:04 PM
 * Author : hoyla
 */ 


#include <avr/io.h>
#include "tinyspi.h"
#include "bitmanip.h"
#include "mrf24j.h"
#include "pktspec.h"
#include "pindefs.h"
#include "simple_queue.h"
#include <string.h>

#define CMD_DATA 0x4441
#define CMD_PING 0x4552
#define CMD_ECHO 0x4543
#define CMD_GET_PARAMETER 0x5047
#define CMD_SET_PARAMETER 0x5053
#define CMD_QSTATUS 0x5153

#define ERR_UNRECOGNIZED_COMMAND 0x0001

#define ADC_N_SAMPLES 128

#define ADC_CS PB7

#define PAN_ID 0xF122

#define PAN_ID_HI 0xF1
#define PAN_ID_LO 0x22

void handle_rx();
void handle_tx();

uint16_t get_adc_value(uint8_t chan);
uint8_t get_packed_data(uint8_t * op, uint8_t ch);

void execute_next_command();

void command_get_data(uint8_t*);
void command_ping(uint8_t*);

void set_downstream_address_header(uint8_t* buff);

void send_downstream(uint8_t* msg);
void send_upstream(uint8_t* msg);

void send_error(uint16_t);

#endif