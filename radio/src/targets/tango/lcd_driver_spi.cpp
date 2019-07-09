/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"

#define WAIT_FOR_DMA_END()             do { } while (lcd_busy)

#define LCD_CS_HIGH()                 LCD_NCS_GPIO->BSRRL = LCD_NCS_GPIO_PIN
#define LCD_CS_LOW()                  LCD_NCS_GPIO->BSRRH = LCD_NCS_GPIO_PIN

#define LCD_RST_HIGH()                LCD_RST_GPIO->BSRRL = LCD_RST_GPIO_PIN
#define LCD_RST_LOW()                 LCD_RST_GPIO->BSRRH = LCD_RST_GPIO_PIN

#define LCD_DC_HIGH()                 LCD_DC_GPIO->BSRRL = LCD_DC_GPIO_PIN
#define LCD_DC_LOW()                  LCD_DC_GPIO->BSRRH = LCD_DC_GPIO_PIN

#define SPI_TIMEOUT					  10000000UL

volatile bool lcd_busy = false;

static void spiWrite(uint8_t byte)
{
  LCD_DC_LOW();
  LCD_CS_LOW();

  uint32_t timeout = 0;

  while ((LCD_SPI->SR & SPI_SR_TXE) == 0 && timeout++ < SPI_TIMEOUT) {
    // Wait
  }

  if(timeout >= SPI_TIMEOUT ){
	  LCD_CS_HIGH();
	  return;
  }

  timeout = 0;

  (void)LCD_SPI->DR; // Clear receive
  LCD_SPI->DR = byte;

  while ((LCD_SPI->SR & SPI_SR_RXNE) == 0 && timeout++ < SPI_TIMEOUT) {
    // Wait
  }

  LCD_CS_HIGH();
}


void spiWriteCommand(uint8_t command)
{
  spiWrite(command);
}

void spiWriteArg(uint8_t arg)
{
  spiWrite(arg);
}

void spiWriteCommandWithArg(uint8_t command, uint8_t arg)
{
  spiWriteCommand(command);
  spiWriteArg(arg);
}

void lcdHardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // APB1 clock / 2 = 133nS per clock
  LCD_SPI->CR1 = 0; // Clear any mode error
  LCD_SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA;
  LCD_SPI->CR2 = 0;
  LCD_SPI->CR1 |= SPI_CR1_MSTR;	// Make sure in case SSM/SSI needed to be set first
  LCD_SPI->CR1 |= SPI_CR1_SPE;

  LCD_CS_HIGH();

  GPIO_InitStructure.GPIO_Pin = LCD_NCS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(LCD_NCS_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_RST_GPIO_PIN;
  GPIO_Init(LCD_RST_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_DC_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_DC_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_CLK_GPIO_PIN | LCD_MOSI_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(LCD_SPI_GPIO, &GPIO_InitStructure);

  GPIO_PinAFConfig(LCD_SPI_GPIO, LCD_MOSI_GPIO_PinSource, LCD_GPIO_AF);
  GPIO_PinAFConfig(LCD_SPI_GPIO, LCD_CLK_GPIO_PinSource, LCD_GPIO_AF);

  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA
  LCD_DMA->HIFCR = LCD_DMA_FLAGS; // Write ones to clear bits
  LCD_DMA_Stream->CR =  DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  LCD_DMA_Stream->PAR = (uint32_t)&LCD_SPI->DR;
#if LCD_W == 128
  LCD_DMA_Stream->NDTR = LCD_W;
#else
  LCD_DMA_Stream->M0AR = (uint32_t)displayBuf;
  LCD_DMA_Stream->NDTR = LCD_W*LCD_H/8*4;
#endif
  LCD_DMA_Stream->FCR = 0x05; // DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;

  NVIC_EnableIRQ(LCD_DMA_Stream_IRQn);
}


extern "C" void LCD_DMA_Stream_IRQHandler()
{
#ifndef BOOT
  CoEnterISR();
#endif
  DEBUG_INTERRUPT(INT_LCD);

  LCD_DMA_Stream->CR &= ~DMA_SxCR_TCIE; // Stop interrupt
  LCD_DMA->HIFCR |= LCD_DMA_FLAG_INT; // Clear interrupt flag
  LCD_SPI->CR2 &= ~SPI_CR2_TXDMAEN;
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA

  while (LCD_SPI->SR & SPI_SR_BSY) {
    /* Wait for SPI to finish sending data
    The DMA TX End interrupt comes two bytes before the end of SPI transmission,
    therefore we have to wait here.
    */
  }

  LCD_DC_LOW(); // command write
  spiWrite(0xAF);
  LCD_CS_HIGH();

  lcd_busy = false;
#ifndef BOOT
  CoExitISR();
#endif
}

/*
  Proper method for turning of LCD module. It must be used,
  otherwise we might damage LCD crystals in the long run!
*/
void lcdOff()
{
  WAIT_FOR_DMA_END();

  /*
  LCD Sleep mode is also good for draining capacitors and enables us
  to re-init LCD without any delay
  */
  spiWriteCommand(0xAE); // LCD sleep
  delay_ms(3); // Wait for caps to drain
}

/*
  Starts LCD initialization routine. It should be called as
  soon as possible after the reset because LCD takes a lot of
  time to properly power-on.
  Make sure that delay_ms() is functional before calling this function!
*/

void lcdSetRefVolt(uint8_t val)
{
}

void lcdDisplayInit(void)
{
  LCD_CS_LOW();
  LCD_DC_LOW();		// command write

  spiWriteCommandWithArg(0x0fd, 0x012),		  /* unlock display, usually not required because the display is unlocked after reset */
  spiWriteCommand(0x0ae),		                /* display off */
  //spiWriteCommandWithArg(0x0a8, 0x03f),		/* multiplex ratio: 0x03f * 1/64 duty */
  //spiWriteCommandWithArg(0x0a8, 0x05f),		/* multiplex ratio: 0x05f * 1/64 duty */
  spiWriteCommandWithArg(0x0a8, 0x07f),     /* multiplex ratio: 0x05f * 1/128duty */
  spiWriteCommandWithArg(0x0a1, 0x000),		  /* display start line */
  //spiWriteCommandWithArg(0x0a2, 0x04c),		/* display offset, shift mapping ram counter */

  spiWriteCommandWithArg(0x0a2, 0x000),		  /* display offset, shift mapping ram counter */
  spiWriteCommandWithArg(0x0a0, 0x51),		  /* remap configuration */


  spiWriteCommandWithArg(0x0ab, 0x001),		  /* Enable internal VDD regulator (RESET) */
  //spiWriteCommandWithArg(0x081, 0x070),		/* contrast, brightness, 0..128 */
  spiWriteCommandWithArg(0x081, 0x053),		  /* contrast, brightness, 0..128 */
  //spiWriteCommandWithArg(0x0b1, 0x055),   /* phase length */
  spiWriteCommandWithArg(0x0b1, 0x051),     /* phase length */
  //spiWriteCommandWithArg(0x0b3, 0x091),		/* set display clock divide ratio/oscillator frequency (set clock as 135 frames/sec) */
  spiWriteCommandWithArg(0x0b3, 0x001),		  /* set display clock divide ratio/oscillator frequency  */

  //? spiWriteCommandWithArg(0x0ad, 0x002),	/* master configuration: disable embedded DC-DC, enable internal VCOMH */
  //? spiWriteCommand(0x086),				        /* full current range (0x084, 0x085, 0x086) */

  spiWriteCommand(0x0b9),				            /* use linear lookup table */

  //spiWriteCommandWithArg(0x0bc, 0x010),   /* pre-charge voltage level */
  spiWriteCommandWithArg(0x0bc, 0x008),     /* pre-charge voltage level */
  //spiWriteCommandWithArg(0x0be, 0x01c),   /* VCOMH voltage */
  spiWriteCommandWithArg(0x0be, 0x007),     /* VCOMH voltage */
  spiWriteCommandWithArg(0x0b6, 0x001),		  /* second precharge */
  spiWriteCommandWithArg(0x0d5, 0x062),		  /* enable second precharge, internal vsl (bit0 = 0) */

  spiWriteCommand(0x0a4);				            /* normal display mode */

  LCD_CS_HIGH();
}

void lcdReset()
{
  TRACE("LCD reset\r\n");
  LCD_RST_HIGH();
  delay_ms(50);
  LCD_RST_LOW(); // module reset
  delay_ms(50);
  LCD_RST_HIGH();
  delay_ms(50);
}

void lcdInit(void)
{
  lcdHardwareInit();

  if (IS_LCD_RESET_NEEDED()) {
    //spiLock(SPI_BUSY_LCD);
    lcdReset();
    lcdDisplayInit(); // panel configure
    //spiUnlock();
  }
}

void lcdRefreshWait()
{
  //spiWait();
  WAIT_FOR_DMA_END();
}

void lcdRefresh(bool wait)
{
  uint8_t * p = displayBuf;
  //spiLock(SPI_BUSY_LCD);

  LCD_CS_LOW();
  LCD_DC_LOW(); // command write

  spiWriteCommand(0x75);
  spiWriteArg(0);
  spiWriteArg(LCD_H - 1);

  spiWriteCommand(0x15);	// set column address
  spiWriteArg(0);
  spiWriteArg((LCD_W / 2) - 1);

  LCD_DC_HIGH(); // data write


  LCD_CS_LOW();;

  lcd_busy = true;
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA
  LCD_DMA->HIFCR = LCD_DMA_FLAGS; // Write ones to clear bits
  LCD_DMA_Stream->M0AR = (uint32_t)p;
  LCD_DMA_Stream->NDTR = 16*32*12;
  LCD_DMA_Stream->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA & TC interrupts
  LCD_SPI->CR2 |= SPI_CR2_TXDMAEN;

  WAIT_FOR_DMA_END();

  if (wait) {
    //spiWait();
  }
}

void lcdTransferTail()
{
  LCD_DC_LOW(); // command write
  spiWrite(0xAF);
  LCD_CS_HIGH();
  //spiEnableMiso();
  //spiUnlock();
}

#if 0
void backlightEnable(uint8_t level)
{
}

void backlightDisable()
{
}

#endif
