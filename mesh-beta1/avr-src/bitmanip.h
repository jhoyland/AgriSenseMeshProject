/*
  bitmanip.h
  Author: James Hoyland
  Abstract: These are some simple utility functions for manipulating data bits. It includes functions for splitting
  two-byte words into bytes and for merging 2 bytes into a single word. Also for packing and unpacking 12 bit values
  into into groups of 3 bytes. 
  
*/

#ifndef __BITMANIP_H
#define __BITMANIP_H


// Provides definitions for uint8_t and uint16_t
#include <stdint.h>

/*Combines two consecutive bytes in an array of uint8_t bytes into a single uint16_t word assumes 
Most significant byte is first. 
@param b: a pointer to the first (MSB) byte. The second byte must follow 
@return: a uint16_t value formed of the two source bytes
*/
uint16_t bytes_to_word(uint8_t* b);
/*Takes a uint16 and splits it between two consecutive bytes
@param b: a pointer to the first (MSB) byte of the result. The second byte must follow 
@param d: The uint8_t word to be split into 
*/
void word_to_bytes(uint8_t* b, uint16_t d);

/*Takes 2  12-bit values stored in uint16_t array (e.g. from an ADC) and packs them into 3 uint8_t bytes
*/
void pack_12bit(uint8_t* dest, uint16_t* src);

/* Unpacks 2 12-bit words from 3 uint8_t bytes
*/
void unpack_12bit(uint16_t* dest, uint8_t* src);


#endif
