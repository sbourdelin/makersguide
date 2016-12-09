/*
 * utilities.c
 *
 * Holds commonly used functions
 */
#include <stdio.h>
#include <stdarg.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "utilities.h"

volatile uint32_t msTicks = 0;

void delay(uint32_t milliseconds)
{
	uint32_t start = msTicks;
	while ((start + milliseconds) > msTicks)
		;
}

// Pass in the elapsed time before it times out, returns the future time
int32_t set_timeout_ms(int32_t timeout_ms)
{
	return msTicks + timeout_ms;
}

// Check to see if the future time has elapsed.
int32_t expired_ms(int32_t timeout_ms)
{
	if (timeout_ms < msTicks)
	{
		return true;
	}
	return false;
}

void set_led(int number, int level)
{
	if (number == 0)
	{
		GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull, level);
	}

	if (number == 1)
	{
		GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull, level);
	}
}


void setup_utilities()
{
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}
}

void SysTick_Handler(void)
{
	msTicks++;
}
