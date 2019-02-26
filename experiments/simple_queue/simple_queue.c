#include "simple_queue.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint8_t* advance_cursor(struct simple_queue*, uint8_t*);

uint8_t setup_queue(struct simple_queue* q, uint8_t n, uint8_t s, uint8_t f)
{
	uint8_t sz = n*s;
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
	for(i=0;i<=sz;i++)
	{
		*x = f;
		x++;
	}

}

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