/*
 * node_statuses.h
 *
 * Created: 3/6/2020 11:43:06 AM
 *  Author: Michael Hilborn
 */ 

#define STATUS_NO_NEIGHBORS			0x0000	//node has no neighbors
#define STATUS_ONE_NEIGHBOR			0x0001	//node has 1 neighbor (final node will have 1)
#define STATUS_TWO_NEIGHBORS		0x0002	//node has 2 neighbors
#define STATUS_STANDBY				0x000A	//node is doing nothing. Alive and listening
#define STATUS_ACQUISITION_WAIT		0x000B	//node has recieved a data acquisition routine request, check other variables before reading
#define STATUS_ACQUISITION_READ		0x000C	//node is entering an ADC read routine
#define STATUS_DATA_TRANSMISSION	0x000D	//node is transmitting data
#define STATUS_SLEEP				0x000E	//node is asleep (shhhhh)