#ifndef __BLINKIN_H
#define __BLINKIN_H

#ifndef BLINK_DELAY
#define BLINK_DELAY 200
#endif

#define BLINK(__BLINK_PORT, __BLINK_PIN) __BLINK_PORT |= (1<<__BLINK_PIN); _delay_ms(BLINK_DELAY); __BLINK_PORT &= ~(1<<__BLINK_PIN); _delay_ms(BLINK_DELAY);  

#endif
