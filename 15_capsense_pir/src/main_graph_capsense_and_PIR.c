/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include "em_device.h"
#include "em_chip.h"
#include <stdio.h>
#include <string.h>

#include "em_cmu.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_prs.h"
#include "em_emu.h"
#include "em_usart.h"

#include "utilities.h"

// Count the number of ACMP pulses
// Global variables
volatile unsigned int count = 0;
volatile bool measurement_complete = false;

#define EM4_WAKEUP_ENABLE_PC9     0x04  // Must change when changing w/u pin

void setup_capsense()
{
	/* Use the default STK capacative sensing setup */
	ACMP_CapsenseInit_TypeDef capsenseInit = ACMP_CAPSENSE_INIT_DEFAULT;

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ACMP1, true);

	/* Set up ACMP1 in capsense mode */
	ACMP_CapsenseInit(ACMP1, &capsenseInit);

	// This is all that is needed to setup PC8, or the left-most slider
	// i.e. no GPIO routes or GPIO clocks need to be configured
	ACMP_CapsenseChannelSet(ACMP1, acmpChannel0);

	// Enable the ACMP1 interrupt
	ACMP_IntEnable(ACMP1, ACMP_IEN_EDGE);
	ACMP1->CTRL = ACMP1->CTRL | ACMP_CTRL_IRISE_ENABLED;

	// Wait until ACMP warms up
	while (!(ACMP1->STATUS & ACMP_STATUS_ACMPACT)) ;

	CMU_ClockEnable(cmuClock_PRS, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Use TIMER1 to count ACMP events (rising edges)
	// It will be clocked by the capture/compare feature
	TIMER_Init_TypeDef timer_settings = TIMER_INIT_DEFAULT;
	timer_settings.clkSel = timerClkSelCC1;
	timer_settings.prescale = timerPrescale1024;
	TIMER_Init(TIMER1, &timer_settings);
	TIMER1->TOP  = 0xFFFF;

	// Set up TIMER1's capture/compare feature, to act as the source clock
	TIMER_InitCC_TypeDef timer_cc_settings = TIMER_INITCC_DEFAULT;
	timer_cc_settings.mode = timerCCModeCapture;
	timer_cc_settings.prsInput = true;
	timer_cc_settings.prsSel = timerPRSSELCh0;
	timer_cc_settings.eventCtrl = timerEventRising;
	timer_cc_settings.edge = timerEdgeBoth;
	TIMER_InitCC(TIMER1, 1, &timer_cc_settings);

	// Set up PRS so that TIMER1 CC1 can observe the event produced by ACMP1
	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_ACMP1, PRS_CH_CTRL_SIGSEL_ACMP1OUT, prsEdgePos);

	// Configure ACMP1 output to a pin D7 at location 2
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);
	ACMP_GPIOSetup(ACMP1, 2, true, false);

	// Set up USART0 for graphing via PC serial port on pins
	CMU_ClockEnable(cmuClock_USART0, true);
	USART_InitAsync_TypeDef usart_settings = USART_INITASYNC_DEFAULT;
	USART_InitAsync(USART0, &usart_settings);

	// Enable TX only at location 5
	USART0->ROUTE = (USART0->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC5;
	USART0->ROUTE |= USART_ROUTE_TXPEN;

	// Set the pin as push/pull in the GPIO
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);

	// Set up TIMER0 for sampling TIMER1
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow */
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV512;
	TIMER0->TOP  = 10;
	TIMER0->IEN  = TIMER_IEN_OF;
	TIMER0->CNT  = 0;

	/* Enable TIMER0 interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn);

	// Set the LED as push/pull
	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);

	// Install the PIR sensor on PC9
	GPIO_PinModeSet(gpioPortC, 9, gpioModeInput, 0);

}

// Prints out the global count variable to the USART
void print_count()
{
	// Create a place to store a message
	char message[6];

	// Format the string as count, encoded in hex, with a space
	sprintf(message, "%x ", count);

	// A string pointer to the start of the message
	char * string = message;

	// While the data pointed to by the string is not null
	while (*string != 0)
	{
		// Send the dereferenced value of the pointer to the USART
		// And then the pointer is incremented to the next address with ++
		USART_Tx(USART0, *string++);
	}
}


void enter_em4(void)
{
	EMU_EM4Init_TypeDef em4_init = EMU_EM4INIT_DEFAULT;
	EMU_EM4Init(&em4_init);

	// Set the wake up enable on PC9 and the polarity of high from the PIR OUT
	GPIO->EM4WUEN = EM4_WAKEUP_ENABLE_PC9;
	GPIO->EM4WUPOL = 1;

	// Set EM4WUCLR = 1 in GPIO CMD, to clear all previous events
	GPIO->CMD = 1;

	EMU_EnterEM4();

	// Now, the system can only be awakened by PC9 or reset, and it acts as
	//   if it is coming out of reset.  Code beyond this point will
	//   never execute.
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	setup_utilities();

	setup_capsense();

	while (1)
	{
		// Check the PIR OUT signal
		if (GPIO_PinInGet(gpioPortC, 9))
		{
			// PIR is high, so set the LED0 to ON
			GPIO_PinOutSet(gpioPortE, 2);
		}
		else
		{
			// PIR is low, so turn off LED0 and enter EM4 engergy mode
			GPIO_PinOutClear(gpioPortE, 2);
			enter_em4();
		}

		// Clear the count and start the timers
		measurement_complete = false;
		TIMER0->CNT = 0;
		TIMER1->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		TIMER1->CMD = TIMER_CMD_START;

		// Now, wait for TIMER0 interrupt to set the complete flag
		while(!measurement_complete)
		{
			EMU_EnterEM1();
		}

		// Now observe the count, send it out the USART
		print_count();

		// Delay to not overwhelm the serial port
		delay(100);
	}
}

void TIMER0_IRQHandler(void)
{
	// Stop timers
	TIMER0->CMD = TIMER_CMD_STOP;
	TIMER1->CMD = TIMER_CMD_STOP;

	// Clear interrupt flag
	TIMER0->IFC = TIMER_IFC_OF;

	// Read out value of TIMER1
	count = TIMER1->CNT;

	measurement_complete = true;
}
