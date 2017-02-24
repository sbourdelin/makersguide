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
#include "em_timer.h"
#include "em_emu.h"

#include <stdio.h>

#define ONE_SECOND_TIMER_COUNT		13672

int _write(int file, const char *ptr, int len)
{
    int x;
    for (x = 0; x < len; x++)
    ITM_SendChar (*ptr++);
    return (len);
}

void SWO_SetupForPrint(void) {
    /* Enable GPIO clock. */
    CMU ->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
    /* Enable Serial wire output pin */
    GPIO ->ROUTE |= GPIO_ROUTE_SWOPEN;
    #if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_LEOPARD_FAMILY) ||         defined(_EFM32_WONDER_FAMILY) || defined(_EFM32_GECKO_FAMILY)
        /* Set location 0 */
        GPIO ->ROUTE = (GPIO ->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) |     GPIO_ROUTE_SWLOCATION_LOC0;
        /* Enable output on pin - GPIO Port F, Pin 2 */
        GPIO ->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
        GPIO ->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
    #else
        /* Set location 1 */
        GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK))         |GPIO_ROUTE_SWLOCATION_LOC1;
        /* Enable output on pin */
        GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
        GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
    #endif
    /* Enable debug clock AUXHFRCO */
    CMU ->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;
    /* Wait until clock is ready */
    while (!(CMU ->STATUS & CMU_STATUS_AUXHFRCORDY)) ;
    /* Enable trace in core debug */
    CoreDebug ->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM ->LAR = 0xC5ACCE55;
    ITM ->TER = 0x0;
    ITM ->TCR = 0x0;
    TPI ->SPPR = 2;
    TPI ->ACPR = 0xf;
    ITM ->TPR = 0x0;
    DWT ->CTRL = 0x400003FE;
    ITM ->TCR = 0x0001000D;
    TPI ->FFCR = 0x00000100;
    ITM ->TER = 0x1;
}

// Just a timer to keep track of time, up to a few seconds
void setup_elapsed_timer()
{
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0)
		;

	// Turn on the overflow interrupt
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Set TIMER Top value to one second
	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);
}

// This timer doesn't keep track of time, but edges on the input pins
void setup_quadrature_timer()
{
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Set CC0 channel defaults
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.edge       = timerEdgeBoth;		// X4 mode
	timerCCInit.filter     = true;

	// Configure CC channel 0
	TIMER_InitCC(TIMER0, 0, &timerCCInit);

	// Configure CC channel 1 in exactly the same way
	TIMER_InitCC(TIMER0, 1, &timerCCInit);

	// Set TIMER0 defaults
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.mode       = timerModeQDec;	// Quadrature mode
	timerInit.quadModeX4 = false;			// X4

	// Configure TIMER0
	TIMER_Init(TIMER0, &timerInit);

	// Route the input pins to TIMER0 from Location 3 and enable CC0 and CC1 in the route
	TIMER0->ROUTE = (TIMER0->ROUTE & ~_TIMER_ROUTE_LOCATION_MASK) | TIMER_ROUTE_LOCATION_LOC3;
	TIMER0->ROUTE |= TIMER_ROUTE_CC0PEN;
	TIMER0->ROUTE |= TIMER_ROUTE_CC1PEN;

	// Enable the GPIO pins needed

	// CC0 on Location 3 is PD1
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInputPullFilter, 1);

	// CC1 on Location 3 is PD2
	GPIO_PinModeSet(gpioPortD, 2, gpioModeInputPullFilter, 1);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

    SWO_SetupForPrint();
    printf("Chapter 6: Input Modes!\n");

    setup_elapsed_timer();
    setup_quadrature_timer();

	while (1)
	{
		// Reset the timers
		TIMER0->CNT = 0;
		TIMER1->CNT = 0;

		// Sleep until the timer expires
		EMU_EnterEM1();

		// One second has passed, fetch the number of detents
		int16_t detents = TIMER0->CNT;

		// Correct for X2 to X1 mode
		detents = detents / 2;

		if (detents != 0)
		{
			// Fetch the direction
			uint8_t direction = (TIMER0->STATUS & 0b10) >> 1;

			// calculate the speed of rotation per minute, RPM
			int16_t rpm = (detents * 60) / 20;

			// Print out the message
			if (direction == 1)
			{
				printf("Shaft Direction:CCW Detents:%d RPM:%d\n", detents, rpm);
			}
			else
			{
				printf("Shaft Direction:CW Detents:%d RPM:%d\n", detents, rpm);
			}
		}
	}
}

// Interrupt handler to clear the overflow
void TIMER1_IRQHandler(void)
{
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
}
