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

#include "em_cmu.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_prs.h"
#include "em_emu.h"

#include "utilities.h"

#define ACMP_PERIOD_MS	100

// Count the number of ACMP pulses
volatile uint32_t count = 0;

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
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	// Setup the systick for 1ms interrupts
	setup_utilities();

	setup_capsense();

	while (1)
	{
		// Clear the count
		count = 0;
		TIMER1->CMD = TIMER_CMD_START;

		// Start a timer based on systick
		int32_t timer = set_timeout_ms(ACMP_PERIOD_MS);

		while (!expired_ms(timer))
		{
			EMU_EnterEM1();
		}

		// Now observe the count and reset
		TIMER1->CMD = TIMER_CMD_STOP;
		count = TIMER1->CNT;
		TIMER1->CNT = 0;

	}
}
