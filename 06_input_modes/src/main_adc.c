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
#include "em_adc.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_emu.h"

#include <stdio.h>

#define BUFFER_SIZE 				16
#define ONE_SECOND_TIMER_COUNT		13672
#define ADC_NOISE 					20

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

void setup_ADC()
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Set the sample rate of ADC0
	ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(7000000, 0);
	ADC_Init(ADC0, &init);

	// Set up the ADC0 on channel0 (PD0) single-ended, referenced to 2.5V internal reference
	// Note that PD0 must be wire jumpered to PC6, since the light sensor is connected to PC6 on the starter kit
	ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;
	sInit.input = adcSingleInputCh0;
	sInit.reference = adcRefVDD; //adcRef2V5;
	sInit.acqTime = adcAcqTime32;
	ADC_InitSingle(ADC0, &sInit);

	// Excite the light sensor on PD6
	GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 1);
}

void setup_timer()
{
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0)
		;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

    SWO_SetupForPrint();
    printf("Chapter 6: Input Modes!\n");

    setup_ADC();
    setup_timer();

	uint32_t value;
	uint32_t last_value = 9999; // Impossible ADC value
	uint32_t sample = 0;

	// Delay a bit to let the ADC warm up
	for (volatile int i=0; i<10000; i++) ;

	while (1)
	{
		// Start an ADC aquisition
		ADC_Start(ADC0, adcStartSingle);

		// Wait for the measurement to complete
		while ( ADC0->STATUS & ADC_STATUS_SINGLEACT);

		// Get the ADC value
		value = ADC_DataSingleGet(ADC0);

		// Only print if the value has changed
		if ((value + ADC_NOISE) < last_value || (value - ADC_NOISE) > last_value)
		{
			float voltage = value * 3.3;
			voltage /= 4096.0;

			// NOTE: To get floating point printing to work, you must enable "print floats" check box in:
			// Project > Properties > C/C++ Build > GNU ARM Linker > General
			printf("Sample #%3d ADC:%4d Voltage:%3.2f\n", (int) sample, (int) value, voltage);
		}

		last_value = value;
		sample++;

		// Wait a second before the next ADC sample
		TIMER1->CNT = 0;
		TIMER1->CMD = TIMER_CMD_START;

		// Sleep until the timer expires.
		EMU_EnterEM1();
	}
}

void TIMER1_IRQHandler(void)
{
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	TIMER1->CMD = TIMER_CMD_STOP;
}

