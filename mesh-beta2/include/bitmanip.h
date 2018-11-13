#ifndef __BITMANIP_H
#define __BITMANIP_H

#include <stdint.h>

uint16_t bytes_to_word(uint8_t* b);
void word_to_bytes(uint8_t* b, uint16_t d);

void pack_12bit(uint8_t* dest, uint16_t* src);
void unpack_12bit(uint16_t* dest, uint8_t* src);


#endif
