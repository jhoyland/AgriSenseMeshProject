//WORK IN PROGRESS

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "mrf24j.h"
#include "mrfpindefs.h"

#define LED_PORT PORTD
#define LED PD4 //local (button) LED light when button is pushed
#define LED2 PD3 //another LED

#define BUTTON PB0
#define BUTTON_PIN PINB

void handle_rx(){//handle the recieved data
	
    if(mrf_get_bufferPHY()){} //don't need, handles entire buffer
		
		uint8_t * rx_data = mrf_get_rxdata();

		if(rx_data[0] == 'a'){
			//if the message received is 'a', blink an LED
		}
		else{
			//blink something else to say 'a' wasn't received
		}
}



uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON)) return 0;
		else return 1;
}

void handle_tx(){
    //chip has sent a message, blink some LED's to let you know
}

//setup
void setup(){
    //everything that runs only once
    //establish which pin does what
    
    //LED is on PD3 and PD4
    DDRD |= (1<<LED); //sets DDRD to output. PD3 is the Pin designation
	DDRD |= (1<<LED2);
    DDRB &= ~(1<<BUTTON); //set DDRB to input. PB0 is pin
   
}


//loop
void loop(){
    mrf_check_flags(&handle_rx, &handle_tx);
    //push button, PB0 is high
    if(pollButton()){
        
    //toggle the LED on and off
    PORTB |= (1<<PD3);
	_delay_ms(1000);
	PORTB &= ~(1<<PD3);
    //send message
    
    }   
    else PORTB |= (0<<PD3);
}

int main(){
    setup();
    while(1) loop();
    return 0;
}
