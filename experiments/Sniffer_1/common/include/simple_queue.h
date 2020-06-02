
#ifndef __SIMPLE_QUEUE_H
#define __SIMPLE_QUEUE_H

/*

Simple queue implements a basic fixed size FIFO queue datastructure.
It stores data as fixed size arrays of bytes. The underlying structure is a ring buffer with two in built cursors.

*/

#include <stdint.h>

struct simple_queue {
	uint8_t* queue;			/* Pointer to the data */
	uint8_t* end;			/* Pointer to the end of the data array (this is not used for storage and just there as a terminator)*/ 
	uint8_t* back;			/* Points to the "back" of the queue - essentailly the last item added */
	uint8_t* front;			/* Points to the front of the queue */
	uint8_t n_elements;		/* Number of elements to be stored */
	uint8_t sz_element;		/* The size of each element in bytes */
};


/* Sets up a new queue
	arguments:
	q: pointer to the queue
	n: number of elements (initializes n_elements)
	s: size of elements (initializes sz_elements)
	f: initialization value for underlying elements (usually 0)

	returns 1 if queue initialized successfully
 */
uint8_t setup_queue(struct simple_queue* q, uint8_t n, uint8_t s, uint8_t f); 

/* Sets up a new queue
	Same as setup_queue except storage is externally managed, storage memory sent via queue_mem argument.
	User must ensure queue_mem array is correct size. should = n*s + 1

*/ 
uint8_t setup_queue_external(struct simple_queue* q, uint8_t* queue_mem, uint8_t n, uint8_t s, uint8_t f);

/* Adds element to the queue 
	arguments:
	q: pointer to the queue to add to
	item: item to add, should be an array of size = sz_element
*/
uint8_t enqueue(struct simple_queue* q, uint8_t* item);

/* Removes an element from the queue 
	arguments:
	q: pointer to the queue to remove item from
	item: array to store the removed item, should be an array of size = sz_element
*/
uint8_t dequeue(struct simple_queue* q, uint8_t* item);

/* Diagnostic print - prints out the queue
*/
void print_queue(struct simple_queue*);

#endif