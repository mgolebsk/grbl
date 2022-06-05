#include "grbl.h"

#define HOLD_LED 0         // POWER ON LED
#define PAPER_IS_OUT_LED 1 // TRASH_BIN_LED
#define ALARM_LED 2        // INK DROP LED
#define NO_LED 3           // LED IS NOT INSTALLED
#define X04_LED 4
#define PAPER_X_IS_OUT_LED 5
#define PAPER_Y_IS_OUT_LED 6
#define PEN_IS_UP_LED 7
// to be used
#define X08_LED 8
#define X09_LED 9
#define X10_LED 10
#define X11_LED 11


#define PAPER_IS_OUT_BIT    bit(0)
#define PAPER_X_IS_OUT_BIT  bit(1)
#define PAPER_Y_IS_OUT_BIT  bit(2)
#define PEN_IS_UP_BIT       bit(3)
#define ALARM_BIT           bit(4)
#define HOLD_BIT            bit(5)


static uint8_t led_state;

void led_reset() {
    led_state = 0;
}

void led_update() {
    bool state = !PAPER_IS_OUT;
    if(xorr(state,  led_state&PAPER_IS_OUT_BIT)) {
        led_state ^= PAPER_IS_OUT_BIT;
        led(PAPER_IS_OUT_LED, state);
    }
    state = !PAPER_X_IS_OUT;
    if(xorr(state, led_state&PAPER_X_IS_OUT_BIT)) {
        led_state ^= PAPER_X_IS_OUT_BIT;
        led(PAPER_X_IS_OUT_LED, state);
    }
    state = !PAPER_Y_IS_OUT;
    if(xorr(state, led_state&PAPER_Y_IS_OUT_BIT)) {
        led_state ^= PAPER_Y_IS_OUT_BIT;
        led(PAPER_Y_IS_OUT_LED, state);
    }
    state = PEN_IS_DOWN;
    if(xorr(state, led_state&PEN_IS_UP_BIT)) {
        led_state ^= PEN_IS_UP_BIT;
        led(PEN_IS_UP_LED, state);
    }
    state = (sys.state & STATE_ALARM);
    if(xorr(state, led_state&ALARM_BIT)) {
        led_state ^= ALARM_BIT;
        led(ALARM_LED, state);
    }
    state = (sys.state & STATE_HOLD);
    if(xorr(state, led_state&HOLD_BIT)) {
        led_state ^= HOLD_BIT;
        led(HOLD_LED, state);
    }
}