#include "main.h"

#include "bsp_led.h"

void LED_toggle(LED_color color,int value){
    switch (color)
    {
    case BLUE:
    if(value)
        GPIOC->BSRR = GPIO_PIN_0 << 16u;
    else
        GPIOC->BSRR = GPIO_PIN_0;
        break;

    case GREEN:
    if(value)
        GPIOC->BSRR = GPIO_PIN_1 << 16u;
    else
        GPIOC->BSRR = GPIO_PIN_1;
    break;

    case RED:
    if(value)
        GPIOC->BSRR = GPIO_PIN_2 << 16u;
    else
        GPIOC->BSRR = GPIO_PIN_2;
    break;

    default:
        break;
    }
    return;

}