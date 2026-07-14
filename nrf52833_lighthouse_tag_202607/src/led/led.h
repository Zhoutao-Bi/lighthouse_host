#ifndef LED_H
#define LED_H

#include <stdint.h>

#define LED_DEFAULT_PERIOD_MS 500U

int led_init(void);
int led_on(void);
int led_off(void);
int led_toggle(void);
int led_blink(uint32_t period_ms);

#endif /* LED_H */
