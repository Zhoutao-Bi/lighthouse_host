#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "led.h"

#define LED_STACK_SIZE 512

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);

static atomic_t period_ms = ATOMIC_INIT(0);
static bool thread_started;
static struct k_thread led_thread;
K_THREAD_STACK_DEFINE(led_stack, LED_STACK_SIZE);

static void led_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint32_t p = atomic_get(&period_ms);

		if (p == 0U) {
			k_msleep(50U);
			continue;
		}

		(void)gpio_pin_toggle_dt(&led);
		k_msleep(p);
	}
}

int led_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return -ENODEV;
	}
	return gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
}

int led_on(void)
{
	return gpio_pin_set_dt(&led, 1);
}

int led_off(void)
{
	return gpio_pin_set_dt(&led, 0);
}

int led_toggle(void)
{
	return gpio_pin_toggle_dt(&led);
}

int led_blink(uint32_t period_ms_in)
{
	atomic_set(&period_ms, period_ms_in);

	if ((period_ms_in != 0U) && !thread_started) {
		k_thread_create(&led_thread, led_stack,
				K_THREAD_STACK_SIZEOF(led_stack),
				led_thread_fn, NULL, NULL, NULL,
				K_PRIO_COOP(1), 0, K_NO_WAIT);
		thread_started = true;
	}

	return 0;
}
