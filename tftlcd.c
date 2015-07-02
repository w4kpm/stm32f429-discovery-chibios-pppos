//=================================================================================================
// STM32F429 LCD TFT Controller driver
// Author : Radoslaw Kwiecien
// e-mail : radek@dxp.pl
// http://en.radzio.dxp.pl/stm32f429idiscovery/
//=================================================================================================
#include "ch.h"
#include "stm32f4xx.h"
#include "gpiof4.h"
//#include "sdram.h"
//=================================================================================================
// LCD configuration - change if you need
//=================================================================================================
#define LCD_WIDTH		800
#define LCD_HEIGHT		480

//mode "800x480-60"
//# D: 29.500 MHz, H: 29.738 kHz, V: 60.00 Hz
//    geometry 800 480 800 480 32 timings 33805 96  24  10  3    72     7 endmode
//                                             hbp hfp vbp  vfp  hsync  vsync                                  



#define HFP 	24   // all of these were found in a standard timings file - not sure if they were right, but they seem to work
#define HSYNC 	72   // however - I was pumping in random data- so it may not work correctly -     
#define HBP 	96   // the data sheet of the LCD panel didn't seem to specify them.

#define VFP 	3
#define VSYNC 	7
#define VBP 	10
//=================================================================================================
// Areas computation
//=================================================================================================
#define ACTIVE_W (HSYNC + LCD_WIDTH + HBP - 1)
#define ACTIVE_H (VSYNC + LCD_HEIGHT + VBP - 1)

#define TOTAL_WIDTH  (HSYNC + HBP + LCD_WIDTH + HFP - 1)
#define TOTAL_HEIGHT (VSYNC + VBP + LCD_HEIGHT + VFP - 1)
/* Pixel width in bytes */
#define PIXELWIDTH 2
//=================================================================================================
//
//=================================================================================================
static  const  GPIO_TypeDef * const GPIOInitTable[] = {
		GPIOC, GPIOB, GPIOA, GPIOA, GPIOB, GPIOG,
		GPIOA, GPIOG, GPIOB, GPIOB, GPIOC, GPIOD,
		GPIOD, GPIOG, GPIOG, GPIOA, GPIOB, GPIOB,
		GPIOA, GPIOC, GPIOF, GPIOG,
		0};

static 	uint8_t const PINInitTable[] = {
		10, 0, 11, 12, 1, 6,
		6, 10, 10, 11, 7, 3,
		6, 11, 12, 3, 8, 9,
		4, 6, 10, 7,
		0};

static  uint8_t const AFInitTable[] = {
		14, 9, 14, 14, 9, 14,
		14, 9, 14, 14, 14, 14,
		14, 14, 9, 14, 14, 14,
		14, 14, 14, 14,
		0};
//=================================================================================================
// LTDC Initialization
//=================================================================================================
void TFTLCD_Init(void)
{
	uint8_t i = 0;
	/* GPIO pin configuration */
	while(GPIOInitTable[i] != 0){
		gpio_conf(GPIOInitTable[i], PINInitTable[i], MODE_AF, TYPE_PUSHPULL, SPEED_100MHz, PULLUP_NONE, AFInitTable[i]);
		i++;
	}
	/* PLL  */
	RCC->PLLSAICFGR = (320 << 6) | (7 << 24) | (4 << 28); /// was 200 = 25mHz 400 = 50mHz 320=40mHz
	/* Enable SAI PLL */
	RCC->CR |= RCC_CR_PLLSAION;
	/* wait for SAI PLL ready */
	while((RCC->CR & RCC_CR_PLLSAIRDY) == 0);
	/* enable clock for LTDC */
	RCC->APB2ENR 	   |= RCC_APB2ENR_LTDCEN;
	/* Synchronization Size Configuration */
	LTDC->SSCR 			= ((HSYNC - 1) << 16) 	| (VSYNC - 1);
	/* Back Porch Configuration */
	LTDC->BPCR 			= ((HBP - 1) << 16) 	| (VBP - 1);
	/* Active Width Configuration */
	LTDC->AWCR 			= (ACTIVE_W << 16) 		| (ACTIVE_H);
	/* Total Width Configuration */
	LTDC->TWCR 			= (TOTAL_WIDTH << 16) 	| (TOTAL_HEIGHT);
	/* Window Horizontal Position Configuration */
	LTDC_Layer1->WHPCR  = HBP | ((HBP + LCD_WIDTH - 1) << 16);
	/* Window Vertical Position Configuration */
	LTDC_Layer1->WVPCR  = VBP | ((VBP + LCD_HEIGHT - 1) << 16);
	/* Pixel Format Configuration */
	LTDC_Layer1->PFCR   = 2;
	/* Color Frame Buffer Address */
	LTDC_Layer1->CFBAR  = 0x20000000;
	/* Color Frame Buffer Length */
	LTDC_Layer1->CFBLR  = ((LCD_WIDTH * PIXELWIDTH) << 16) | ((LCD_WIDTH * PIXELWIDTH) + 3);
	/* Enable Layer */
	LTDC_Layer1->CR 	= LTDC_LxCR_LEN;
	/* Immediate Reload */
	LTDC->SRCR 			= LTDC_SRCR_IMR;
	/* Enable LTDC */
	LTDC->GCR  			= LTDC_GCR_LTDCEN;
}
//=================================================================================================
// Function for drawing single pixel
//=================================================================================================
/*void PutPixel(uint16_t x, uint16_t y, uint16_t color)
{
	*((uint16_t *) (SDRAM_BASE + (x * PIXELWIDTH) + (y * (LCD_WIDTH * PIXELWIDTH)))) = color;
	}*/
//=================================================================================================
//
//=================================================================================================
