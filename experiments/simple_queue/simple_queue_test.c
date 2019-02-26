#include "simple_queue.h"
#include <stdlib.h>


int main()
{
	struct simple_queue cmd_queue;

	setup_queue(&cmd_queue, 4, 4, 0x23);

	uint8_t x= 0x41;

	uint8_t xout[4];
	uint8_t yout[4];

	memset(xout,x,4);

	int r,i;
	printf("\nInitial status:\n");
	print_queue(&cmd_queue);
	uint8_t y = 0;
	uint8_t ret;
	printf("\n========\n");

	for(i=0; i<20; i++)
	{
		printf("\nStep %d",i);
		r = rand();
		if(r<RAND_MAX/2)
		{
			ret = enqueue(&cmd_queue,xout);
			if(ret)
			{
				printf("\nEnqueued:%c",(char)(x));
				x++;
				memset(xout,x,4);
			}
			else
			{
				printf("\nEnqueue failed");
			}
		}
		else
		{
			ret = dequeue(&cmd_queue,yout);
			if(ret)
			{
				printf("\nDequeued:%c",(char)(yout[0]));
			}
			else
			{
				printf("\nDequeue failed");
			}
		}
		print_queue(&cmd_queue);
		printf("\n========\n\n");
	}
}
