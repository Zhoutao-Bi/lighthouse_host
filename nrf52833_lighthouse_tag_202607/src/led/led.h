#ifndef LED_H
#define LED_H

#include <stdint.h>

enum led_mode {
	LED_MODE_OFF = 0,
	LED_MODE_INIT,
	LED_MODE_BROADCASTING,
	LED_MODE_ERROR,
};

int  led_init(void);
int  led_on(void);
int  led_off(void);
int  led_toggle(void);
void led_set_mode(enum led_mode mode);
void led_notify_rx(void);

#endif /* LED_H */
