
#ifndef __SIMPLE_QUEUE_H
#define __SIMPLE_QUEUE_H

#include <stdint.h>

struct simple_queue {
	uint8_t* queue;
	uint8_t* end;
	uint8_t* back;
	uint8_t* front;
	uint8_t n_elements;
	uint8_t sz_element;
};

uint8_t setup_queue(struct simple_queue* q, uint8_t n, uint8_t s, uint8_t f);
uint8_t enqueue(struct simple_queue* q, uint8_t* item);
uint8_t dequeue(struct simple_queue* q, uint8_t* item);
void print_queue(struct simple_queue*);

#endif