#include <zephyr/kernel.h>

#include "led.h"

int main(void)
{
	int err = led_init();

	if (err) {
		return err;
	}

	return led_blink(LED_DEFAULT_PERIOD_MS);
}
