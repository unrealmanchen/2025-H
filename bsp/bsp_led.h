#ifndef LED
#define LED

typedef enum{
    RED=0,
    BLUE,
    GREEN
}  LED_color;


void LED_toggle(LED_color color,int value);
#endif
