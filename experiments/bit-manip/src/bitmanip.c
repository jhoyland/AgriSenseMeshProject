#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>


uint8_t packet[127];

uint16_t two_bytes(uint8_t* b)
{
    uint16_t hi = (uint16_t) b[0];
    uint16_t lo = (uint16_t) b[1];

    return (hi << 8) | lo;
}

void two_bytes_out(uint8_t* b, uint16_t d)
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

int main(void)
{
	uint16_t data1[4];
	data1[0] = 2345;
	data1[1] = 678;
	data1[2] = 3210;
	data1[3] = 456;
	
	uint8_t packed1[6];
	
	pack_12bit(packed1,data1);
	pack_12bit(& packed1[3], & data1[2]);
	
	uint16_t unpacked1[4];
	
	unpack_12bit(unpacked1,packed1);
	unpack_12bit(& unpacked1[2], & packed1[3]);

	printf("Packing 12 bit value pairs into bytes");
	
	int i = 0;
	
	printf("\nData:");
	for(i=0;i<4;i++) printf("\n%02d -> %4d (0x%04x)",i,data1[i],data1[i]);
	printf("\n\nPacked:");
	for(i=0;i<6;i++) printf("\n%02d -> 0x%02x",i,packed1[i]);
	printf("\n\nUnpacked:");
	for(i=0;i<4;i++) printf("\n%02d -> %4d (0x%04x)",i,unpacked1[i],unpacked1[i]);
	
	
	uint16_t number = 17248;
	uint8_t bytes[2];
	
	two_bytes_out(bytes,number);
	
	uint16_t number2;
	
	number2 = two_bytes(bytes);
	
	printf("\n\n\nSplitting and joining words and bytes");
	printf("\nOriginal number = %d (0x%04x)",number,number);
	printf("\nSplit bytes = %d , %d  (0x%02x , 0x%02x)",bytes[0], bytes[1], bytes[0], bytes[1]);
	printf("\nRejoined number = %d (0x%04x)",number2,number2);
	 
	return 0;
	
}
	
	

