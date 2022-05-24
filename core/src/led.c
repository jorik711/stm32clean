#include "led.h"

uint8_t num1 = 0, num2 = 0, num3 = 0, num4 = 0; /* номер индикатора */
uint16_t showNumber = 0; /* выводимое число */

void segchar(uint8_t digit) {
    uint8_t r;
    switch (digit)
    {
    case 0:
        r = 0xC0;
        break;
    case 1:
        r = 0xF9;
        break;
    case 2:
        r = 0xA4;
        break;
    case 3:
        r =0xB0;
        break;
    case 4:
        r = 0x99;
        break;
    case 5:
        r = 0x92;
        break;
    case 6:
        r = 0x82;
        break;
    case 7:
        r = 0xF8;
        break;
    case 8:
        r = 0x80;
        break;
    case 9:
        r = 0x90;
        break;
    case 10:
        r = 0xFF;
        break;
    default:
        break;
    }
    //GPIOG->ODR &= ~(r << 4);
    //GPIOG->ODR |= (r << 4);
    MODIFY_REG(GPIOG->ODR, r << 4, ~(r << 4));
   
}

void ledstream(uint16_t number) {
    showNumber = number;
    num1 = number%10;
    num2 = number%100/10;
    num3 = number%1000/100;
    num4 = number%1000;
}