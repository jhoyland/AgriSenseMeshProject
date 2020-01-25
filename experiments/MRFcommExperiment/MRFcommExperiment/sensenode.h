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
#include "cmdspec.h"
#include "netspec.h"
#include "simple_queue.h"
#include <string.h>


//#define ADC_N_SAMPLES 128

void handle_rx();
void handle_tx();

uint16_t get_adc_value(uint8_t chan);
uint8_t get_packed_data(uint8_t * op, uint8_t ch);

void execute_next_command();

void command_get_data(uint8_t*);
void command_ping(uint8_t*);

void command_send_test();
void command_set_parameter(uint8_t*);
void command_get_parameter(uint8_t*);

void set_downstream_address_header(uint8_t* buff);

void send_downstream(uint8_t* msg);
void send_upstream(uint8_t* msg);

void setup();
void setup_ports();
void loop();

void send_error(uint16_t);

#endif