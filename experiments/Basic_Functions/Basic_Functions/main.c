/*
 * Basic_Functions.c
 *
 * Created: 9/24/2019 1:27:11 PM
 * Author : michael
 */ 

#include <avr/io.h>
#include <util/delay.h>

void setup(){ //set the pins you want as outputs
	
	DDRD |= (1<<PD7);
	DDRD |= (1<<PD6);
	DDRD |= (1<<PD5);
}

void loop(){
	PORTD |= (1<<PD7);
	_delay_ms(1000);
	PORTD &= ~(1<<PD7);
	
	PORTD|= (1<<PD6);
	_delay_ms(1000);
	PORTD &= ~(1<<PD6);
}

int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
		loop();
    }
	return 0;
}

