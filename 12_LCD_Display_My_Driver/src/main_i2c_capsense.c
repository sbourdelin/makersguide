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
#include "em_i2c.h"

#include "ili9341.h"
#include "utilities.h"

#define FT6206_ADDR           	0x38 << 1
#define FT6206_FIRMID_OFFSET	0xA6

// Used for our timer
extern uint32_t msTicks;

uint8_t data_array[16];

void peripheral_setup()
{
	// Set up the necessary peripheral clocks
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);
	CMU_ClockEnable(cmuClock_I2C0, true);

	// Initialize and enable the USART
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.baudrate = 24000000;		// This is the fastest it can go
	init.msbf = true;

	// This will get the HFPER clock running at 48MHz
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
//	uint32_t foo = CMU_ClockFreqGet(cmuClock_HFPER);
//	foo = CMU_ClockFreqGet(cmuClock_HF);
//	foo = CMU_ClockFreqGet(cmuClock_CORE);

	USART_InitSync(USART1, &init);

	uint32_t baud = USART_BaudrateGet(USART1);

	USART1->CTRL |= USART_CTRL_AUTOCS;

	// Connect the USART signals to the GPIO peripheral
	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
			USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_LOCATION_LOC1;

	// Set up i2c
	I2C_Init_TypeDef i2c_init = I2C_INIT_DEFAULT;
	I2C_Init(I2C0, &i2c_init);

	// Use I2C0 location #1
	I2C0->ROUTE = (I2C0->ROUTE & ~_I2C_ROUTE_LOCATION_MASK) | I2C_ROUTE_LOCATION_LOC1;

	// Route the pins through to the GPIO block, so that I2C block can control them
	I2C0->ROUTE |= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN;

	// Enable the GPIO pins for the USART, starting with CS lines
	// This is to avoid clocking the flash chip when we set CLK high
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);		// CS
	GPIO_PinModeSet(gpioPortD, 8, gpioModePushPull, 1);		// MicroSD CS
	GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);		// MOSI
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);		// MISO
	GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 1);		// CLK

	// Enable the GPIO pins for the misc signals, leave pulled high
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 1);		// DC
	GPIO_PinModeSet(gpioPortD, 5, gpioModePushPull, 1);		// RST

	// Enable the GPIO pins for the i2c signals, open drain, pulled up, with filter
	GPIO_PinModeSet(gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1);		// SDA
	GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1);		// SCL
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

void set_drawing_window(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
	if (x_start > ILI9341_TFTWIDTH || x_end > ILI9341_TFTWIDTH || y_start > ILI9341_TFTHEIGHT || y_end > ILI9341_TFTHEIGHT)
	{
		DEBUG_BREAK
	}

	// Swap things if they are backwards
	if (x_start > x_end)
	{
		uint16_t tmp;
		tmp = x_end;
		x_end = x_start;
		x_start = tmp;
	}

	// Swap things if they are backwards
	if (y_start > y_end)
	{
		uint16_t tmp;
		tmp = y_end;
		y_end = y_start;
		y_start = tmp;
	}

	writecommand(ILI9341_CASET); // Column addr set
	writedata(x_start >> 8);
	writedata(x_start);
	writedata(x_end >> 8);
	writedata(x_end );

	writecommand(ILI9341_PASET); // Page (row) addr set
	writedata(y_start >> 8);
	writedata(y_start);
	writedata(y_end >> 8);
	writedata(y_end);
}

void draw_filled_rectangle(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
	set_drawing_window(x_start, y_start, x_end, y_end);
	writecommand(ILI9341_RAMWR);
	for (int i = 0; i < x_end; i++)
	{
		for (int j = 0; j < y_end; j++)
		{
			writedata( color >> 8);
			writedata( color );
		}
	}
}

void draw_vertical_line(uint16_t x_start, uint16_t y_start, uint16_t length, int16_t thickness, uint16_t color)
{
	uint16_t x_end = x_start + thickness;
	uint16_t y_end = y_start + length;
	draw_filled_rectangle(x_start, y_start, x_end, y_end, color);
}

void draw_horizontal_line(uint16_t x_start, uint16_t y_start, uint16_t length, int16_t thickness, uint16_t color)
{
	uint16_t x_end = x_start + length;
	uint16_t y_end = y_start + thickness;
	draw_filled_rectangle(x_start, y_start, x_end, y_end, color);
}

void draw_unfilled_rectangle(uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height, uint16_t line_thickness, uint16_t color)
{
	// Draw left side
	draw_vertical_line(x_start, y_start, height, line_thickness, color);

	// Draw bottom side
	draw_horizontal_line(x_start, y_start+height, width, -line_thickness, color);

	// Draw top
	draw_horizontal_line(x_start, y_start, width, line_thickness, color);

	// Draw right side
	draw_vertical_line(x_start+width, y_start, height, -line_thickness, color);
}

void draw_vertical_line_old(uint16_t x_start, uint16_t y_start, uint16_t y_end, uint16_t color)
{
	set_drawing_window(x_start, y_start, x_start, y_end);
	uint16_t length = y_end - y_start + 1;
	writecommand(ILI9341_RAMWR);
	for (int j = 0; j < length; j++)
	{
		writedata( color >> 8);
		writedata( color );
	}
}

// Used by the read_register and write_register functions
// data_array is read or write data, depending on the flag
void i2c_transfer(uint16_t device_addr, uint8_t data_array[], uint16_t data_len, uint8_t flag)
{
	// Transfer structure
	I2C_TransferSeq_TypeDef i2cTransfer;

	// Initialize I2C transfer
	I2C_TransferReturn_TypeDef result;
	i2cTransfer.addr          = device_addr;
	i2cTransfer.flags         = flag;
	i2cTransfer.buf[0].data   = data_array;
	i2cTransfer.buf[0].len    = data_len;

	// Set up the transfer
	result = I2C_TransferInit(I2C0, &i2cTransfer);

	// Do it until the transfer is done
	while (result != i2cTransferDone)
	{
		if (result != i2cTransferInProgress)
		{
			DEBUG_BREAK;
		}
		result = I2C_Transfer(I2C0);
	}
}

// Tailored for the FT6x06 device only
void i2c_read_device_id()
{
	// First, set the address to read
	data_array[0] = FT6206_FIRMID_OFFSET;
	i2c_transfer(FT6206_ADDR, data_array, 1, I2C_FLAG_WRITE);

	// Then, do the actual read of the register contents, from the offset +9
	i2c_transfer(FT6206_ADDR, data_array, 9, I2C_FLAG_READ);

	// Check FT6206_FIRMID_OFFSET + 2 for FocalTech’s Panel ID = 0x11
	if (data_array[2] != 0x11)
	{
		DEBUG_BREAK
	}
	return;
}

void i2c_read_capacitance_values()
{
	// First, set the address to read
	data_array[0] = 0;
	i2c_transfer(FT6206_ADDR, data_array, 1, I2C_FLAG_WRITE);

	// Then, do the actual read of the register contents, from the offset +15
	i2c_transfer(FT6206_ADDR, data_array, 15, I2C_FLAG_READ);
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

	peripheral_setup();

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
	uint32_t color = ILI9341_BLUE;

	uint16_t x_start = 48;
	uint16_t x_end = ILI9341_TFTWIDTH-48;
	uint16_t y_start = 64;
	uint16_t y_end = ILI9341_TFTHEIGHT-64;

	set_drawing_window(x_start, y_start, x_end, y_end );

	writecommand(ILI9341_RAMWR);			//Memory write

	uint32_t elapsed_ms = msTicks;
	for (int i = 0; i < x_end; i++)
	{
		for (int j = 0; j < y_end; j++)
		{
			writedata( color >> 8);
			writedata( color );
		}
	}
	elapsed_ms = msTicks - elapsed_ms;

	draw_vertical_line(48, 0, 320, 1, ILI9341_RED);

	draw_vertical_line(48, 0, 320, 10, ILI9341_RED);

	draw_vertical_line(88, 0, 320, 10, ILI9341_RED);

	draw_horizontal_line(0, 160, 240, 30, ILI9341_GREEN);

	draw_horizontal_line(40, 200, 160, 50, ILI9341_YELLOW);

	draw_unfilled_rectangle(10, 10, 220, 300, 30, ILI9341_PURPLE);

	// Check to see that we can talk to the i2c capacitive sensor
	i2c_read_device_id();

	/* Infinite loop */
	while (1) {
	}


}
