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
#include "em_acmp.h"
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

// Setup the ACMP to output to pin E2, with internal negative reference
void setup_ACMP()
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_ACMP0, true);

	ACMP_Init_TypeDef acmp_init = ACMP_INIT_DEFAULT;
	acmp_init.fullBias = true;
	acmp_init.hysteresisLevel = acmpHysteresisLevel7;
	acmp_init.vddLevel = 24;   // This value should be set just above the room ambient light voltage level
	// It was as low as 10 in a dim room, as high as 24 in a bright room

	/* Init and set ACMP0 channel on PC6 */
	ACMP_Init(ACMP0, &acmp_init);

	/* Init and set ACMP0 channel on PC6 */
	ACMP_Init(ACMP0, &acmp_init);
	ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);

	/* Set up GPIO as output on location 1, which is PE2, without inverting */
	ACMP_GPIOSetup(ACMP0, 1, true, false);
	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);

	/* Wait for warmup */
	while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));

}

// Blink an LED 10 times a second on PD1.  Excite light sensor on PD6
void setup_blink_timer()
{
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER0_IRQn);

	// Set TIMER Top value
	TIMER_TopSet(TIMER0, ONE_SECOND_TIMER_COUNT / 10);

	TIMER_Init(TIMER0, &timerInit);

	// Wait for the timer to get going
	while (TIMER0->CNT == 0)
		;

	// Excite the light sensor on PD6
	GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 1);

	// Set up a GPIO output pin to push pull to blink an LED
	GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

    SWO_SetupForPrint();
    printf("Chapter 6: Input Modes!\n");

    setup_blink_timer();
    setup_ACMP();

	while (1)
	{
		// Sleep until the timer expires
		EMU_EnterEM1();
	}
}

void TIMER0_IRQHandler(void)
{
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	GPIO_PinOutToggle(gpioPortD, 1);
}

