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
#include "em_usart.h"
#include "em_cmu.h"

#include "ili9341.h"
#include "utilities.h"

// Used for our timer
extern uint32_t msTicks;

void usart_setup()
{
	// Set up the necessary peripheral clocks
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	// Enable the GPIO pins for the USART, starting with CS
	// This is to avoid clocking the flash chip when we set CLK high
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);		// CS
	GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);		// MOSI
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);		// MISO
	GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 1);		// CLK

	// Enable the GPIO pins for the misc signals, leave pulled high
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 1);		// DC
	GPIO_PinModeSet(gpioPortD, 5, gpioModePushPull, 1);		// RST

	// Initialize and enable the USART
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.clockMode = usartClockMode0;
	//init.baudrate = 24000000;		// This is the fastest it can go
	init.msbf = true;

	// This will get the HFPER clock running at 48MHz
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
//	uint32_t foo = CMU_ClockFreqGet(cmuClock_HFPER);
//	foo = CMU_ClockFreqGet(cmuClock_HF);
//	foo = CMU_ClockFreqGet(cmuClock_CORE);

	USART_InitSync(USART1, &init);

	//uint32_t baud = USART_BaudrateGet(USART1);

	USART1->CTRL |= USART_CTRL_AUTOCS;

	// Connect the USART signals to the GPIO peripheral
	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
			USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_LOCATION_LOC1;
}

void writecommand(uint8_t command)
{
	delay(1);
	GPIO_PinOutClear(gpioPortD, 4);
	USART_Tx(USART1, command);
	delay(1);
}

void writedata(uint8_t data)
{
	GPIO_PinOutSet(gpioPortD, 4);
	USART_Tx(USART1, data);

}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9341_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	usart_setup();

	// Reset the display driver chip
	GPIO_PinOutSet(gpioPortD, 5);
	GPIO_PinOutClear(gpioPortD, 5);
	delay(1);
	GPIO_PinOutSet(gpioPortD, 5);

	delay(10);
	writecommand(ILI9341_SLPOUT);    //Exit Sleep
	delay(120);
	writecommand(ILI9341_DISPON);    //Display on

	writecommand(ILI9341_PIXFMT);
	writedata(0x55);

	writecommand(ILI9341_MADCTL);    // Memory Access Control
	writedata(0x48);

	//setAddrWindow(50,50,ILI9341_TFTWIDTH-1,ILI9341_TFTHEIGHT-1);
	uint32_t color = ILI9341_RED;

	writecommand(0x002C);			//Memory write

	uint32_t elapsed_ms = msTicks;
	for (int i = 0; i < (ILI9341_TFTWIDTH); i++)
	{
		for (int j = 0; j < ILI9341_TFTHEIGHT; j++)
		{
			writedata( color >> 8);
			writedata( color );
		}
	}
	elapsed_ms = msTicks - elapsed_ms;

	/* Infinite loop */
	while (1) {
	}


}
