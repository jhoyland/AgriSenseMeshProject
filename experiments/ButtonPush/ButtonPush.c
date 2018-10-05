//WORK IN PROGRESS

#include <avr/io.h>
#include <util/delay.h>
#include "mrf24j.h"
#include "mrfpindefs.h"

#define LED_PORT PORTD
#define LED PD3

#define BUTTON PB0

void hande_rx(){//handle the recieved data
    if(mrf_get_bufferPHY()){} //don't need, handles entire buffer
}

unint8_t * rx_data = mrf_get_rxdata();

if(rx_data[0] == 'a'){
    //if the message recieved is 'a', blink an LED
}
else{
    //blink something else to say 'a' wasn't recieved
}

uint8_t pollButton()
{
	if(BUTTON_PIN & (1<<BUTTON_1)) return 0;
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
    DDRD |= (1<<PD3); //sets DDRD to output. PD3 is the Pin designation
    DDRB &= ~(1<<PB0); //set DDRB to input. PB0 is pin
    PORTB |= (1<<PB0);
}


//loop
void loop(){
    mrf_check_flags(&handle_rx, &handle_tx);
    //push button, PB0 is high
    if(pollButton()){
        
    //toggle the LED on and off
    PORTB |= (1<<PD3);
    //send message
    
    }   
    else PORTB |= (0<<PD3);
}

int main(){
    setup();
    while(1) loop();
    return 0;
}
