#ifndef __LED_H
#define __LED_H

#include "stdint.h"
#include "stm32f4xx.h"

void ledstream(uint16_t number);
void segchar(uint8_t digit);


#endif /* __LED_H */