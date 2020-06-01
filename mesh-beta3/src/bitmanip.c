/*
bitmanip.c :- various functions for manipulating bytes
*/

#include <stdint.h>

uint16_t bytes_to_word(uint8_t* b)
{
    uint16_t hi = (uint16_t) b[0];
    uint16_t lo = (uint16_t) b[1];

    return (hi << 8) | lo;
}

void word_to_bytes(uint8_t* b, uint16_t d)
{
    b[0] = (uint8_t)(d >> 8);
    b[1] = (uint8_t)(255 & d);
}

void pack_12bit(uint8_t* dest, uint16_t* src)
{   
    dest[0] = 255 & (src[0] >> 4);
    dest[1] = 255 & ((src[0] << 4) | (src[1] >> 8));
    dest[2] = 255 & src[1];
}

void unpack_12bit(uint16_t* dest, uint8_t* src)
{
	uint16_t src16[3];
	src16[0] = src[0];
	src16[1] = src[1];
	src16[2] = src[2];	
	
	dest[0] = (src16[0] << 4) | (src16[1] >> 4);
	dest[1] = ((15 & src16[1]) << 8) | (src16[2]);
}
