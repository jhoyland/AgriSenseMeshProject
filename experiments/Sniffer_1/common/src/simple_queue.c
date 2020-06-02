#include "simple_queue.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


/*
	The "queue" is really just a simple array of n_elements * sz_elements + 1 bytes.
	Queue type behaviour is provided by 2 cursors - front and back. back points to the next available slot and front points to the slot currently at the font of the queue.

	When initially empty, back points to the start of the array and front is NULL. This latter condition is the marker that the queue is empty
	When an element is added it is copied to "back", back is advanced one sz_element and front points at back.

INITIAL CONDITION

	back
	v
	****|****|****|****|****

	front = NULL

ELEMENT ADDED "AAAA"

		 back
		 v
	AAAA|****|****|****|****
	^
	front


ELEMENT ADDED "BBBB"

		 	  back
		 	  v
	AAAA|BBBB|****|****|****
	^
	front
	
ELEMENT REMOVED

		 	  back
		 	  v
	****|BBBB|****|****|****
		 ^
		 front

ELEMENTS ADDED "CCCC", "DDDD", "EEEE"

"back" cursor wraps back round to beginning

	back
	v
	****|BBBB|CCCC|DDDD|EEEE
		 ^
		 front


ELEMENTS ADDED "FFFF"
queue is now full, back set to NULL

	back=0
	
	FFFF|BBBB|CCCC|DDDD|EEEE
		 ^
		 front

ELEMENT REMOVED
queue is no long full, previous front slot becomes available

		 back
		 V
	FFFF|****|CCCC|DDDD|EEEE
		 	  ^
		 	  front


*/






uint8_t* advance_cursor(struct simple_queue*, uint8_t*);

uint8_t setup_queue(struct simple_queue* q, uint8_t n, uint8_t s, uint8_t f)
{
	uint8_t sz = n*s;
	if(!sz) return 0;
	uint8_t * new_queue = (uint8_t*) malloc((sz + 1) * sizeof(uint8_t));
	if(new_queue == NULL) return 0;
	(*q).queue = new_queue;
	(*q).end   = & new_queue[sz];
	(*q).back  = new_queue;
	(*q).front = NULL;
	(*q).n_elements = n;
	(*q).sz_element = s;
	int i;
	uint8_t* x = new_queue;
	while(x!=(*q).end)
	{
		*x = f;
		x++;
	}

	return 1;

}



uint8_t setup_queue_external(struct simple_queue* q, uint8_t* queue_mem, uint8_t n, uint8_t s, uint8_t f)
{
	uint8_t sz = n*s;
	(*q).queue = queue_mem;
	(*q).end   = & queue_mem[sz];
	(*q).back  = queue_mem;
	(*q).front = NULL;
	(*q).n_elements = n;
	(*q).sz_element = s;
	int i;
	uint8_t* x = queue_mem;
	for(i=0;i<=sz;i++)
	{
		*x = f;
		x++;
	}

	return 1;

}

/*
Advances either of the cursors wrapping them - should not be called directly	
*/

uint8_t* advance_cursor(struct simple_queue* q, uint8_t* curr)
{
	curr += (*q).sz_element;
	if(curr == (*q).end) curr = (*q).queue;
	return curr;
}

uint8_t enqueue(struct simple_queue* q, uint8_t* item)
{
	if((*q).back == NULL) return 0; /*No room*/

	memcpy((*q).back, item, (*q).sz_element*sizeof(uint8_t));

	if((*q).front == NULL) (*q).front = (*q).back; /* The queue was empty before this item was added. This item is now the front of the queue */ 

	(*q).back = advance_cursor(q,(*q).back); /* Move on the back of the queue */

	if((*q).back == (*q).front) (*q).back = NULL; /* Adding the item made the queue full */ 

	return 1;
} 

uint8_t dequeue(struct simple_queue* q, uint8_t* item)
{
	if((*q).front == NULL) return 0; /* Nothing to dequeue */

	memcpy(item,(*q).front, (*q).sz_element*sizeof(uint8_t));

	if((*q).back == NULL) (*q).back = (*q).front; /* The queue was full before this item was removed. The queue now has a space where this item was */
	(*q).front = advance_cursor(q,(*q).front);

	if((*q).back == (*q).front) /* The item was the last in the queue. Reset front / back pointers to start values */
	{
		(*q).front = NULL;
		(*q).back = (*q).queue;
	}	

	return 1;
}

void print_queue(struct simple_queue* q)
{
	uint8_t* curr =(*q).queue;

	printf("\nQ:\n");

	while(curr != (*q).end)
	{
		printf("%c",(char)(*curr));
		curr++;
	}

	printf(".FB\n");

	curr =(*q).queue;

	while(curr != (*q).end)
	{
		if(curr == (*q).front) 
		{
			if(curr == (*q).back)
				printf("*");
			else
				printf("f");
		}
		else
		{
			if(curr == (*q).back)
				printf("b");
			else
				printf("-");
		}
		curr++;
	}

	printf(".");

	if((*q).front == NULL)
		printf("0");
	else
		printf("1");


	if((*q).back == NULL)
		printf("0");
	else
		printf("1");
}