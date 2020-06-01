#include <avr/io.h>
#include <util/delay.h>

//setup
void setup(){
    //everything that runs only once
    //establish which pin does what
    
    //LED is on PD3 and PD4
    DDRD |= (1<<PD3); //sets DDRD to output. PD3 is the Pin designation
}
//loop
void loop(){
    //toggle the LED on and off
    PORTD |= (1<<PD3);//turns 
    _delay_ms(1000);
    
    PORTD &= ~(1<<PD3);//turns off
   _delay_ms(1000);
}

int main(){
    setup();
    while(1) loop();
    return 0;
}
