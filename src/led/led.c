#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "led.h"

#define LED_STACK_SIZE 512
#define LED_PULSE_MS   200
#define LED_PULSE_TICK_MS 50

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);

static atomic_t current_mode = ATOMIC_INIT(LED_MODE_OFF);
static atomic_t pulse_remaining_ms = ATOMIC_INIT(0);
static bool thread_started;
static struct k_thread led_thread;
K_THREAD_STACK_DEFINE(led_stack, LED_STACK_SIZE);

static void pulse_timer_handler(struct k_timer *t);
K_TIMER_DEFINE(pulse_timer, pulse_timer_handler, NULL);

static void pulse_off_handler(struct k_work *w);
K_WORK_DEFINE(pulse_off_work, pulse_off_handler);

static void pulse_off_handler(struct k_work *w)
{
	atomic_set(&pulse_remaining_ms, 0);
}

static void pulse_timer_handler(struct k_timer *t)
{
	k_work_submit(&pulse_off_work);
}

static void led_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		int pulse = atomic_get(&pulse_remaining_ms);
		enum led_mode m = (enum led_mode)atomic_get(&current_mode);

		if (pulse > 0) {
			(void)gpio_pin_toggle_dt(&led);
			k_msleep(LED_PULSE_TICK_MS);
			continue;
		}

		switch (m) {
		case LED_MODE_INIT:
			(void)gpio_pin_toggle_dt(&led);
			k_msleep(200);
			break;
		case LED_MODE_BROADCASTING:
			(void)gpio_pin_toggle_dt(&led);
			k_msleep(500);
			break;
		case LED_MODE_ERROR:
			(void)gpio_pin_set_dt(&led, 1);
			k_msleep(1000);
			break;
		case LED_MODE_OFF:
		default:
			(void)gpio_pin_set_dt(&led, 0);
			k_msleep(100);
			break;
		}
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

void led_set_mode(enum led_mode mode)
{
	atomic_set(&current_mode, (atomic_t)mode);
}

void led_notify_rx(void)
{
	atomic_set(&pulse_remaining_ms, LED_PULSE_MS);
	k_timer_start(&pulse_timer, K_MSEC(LED_PULSE_MS), K_NO_WAIT);

	if (!thread_started) {
		k_thread_create(&led_thread, led_stack,
				K_THREAD_STACK_SIZEOF(led_stack),
				led_thread_fn, NULL, NULL, NULL,
				K_PRIO_COOP(1), 0, K_NO_WAIT);
		thread_started = true;
	}
}
