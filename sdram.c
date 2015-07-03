//=================================================================================================
// STM32F429I-Discovery SDRAM configuration
// Author : Radoslaw Kwiecien
// e-mail : radek@dxp.pl
// http://en.radzio.dxp.pl/stm32f429idiscovery/
// Date : 24.11.2013
//=================================================================================================
#include "ch.h"
#include "stm32f4xx.h"
#include "gpiof4.h"
#include "sdram.h"
//=================================================================================================
// Macros for SDRAM Timing Register
//=================================================================================================
#define TMRD(x) (x << 0)
#define TXSR(x) (x << 4)
#define TRAS(x) (x << 8)
#define TRC(x)  (x << 12)
#define TWR(x)  (x << 16)
#define TRP(x)  (x << 20)
#define TRCD(x) (x << 24)
//=================================================================================================
// GPIO configuration data
//=================================================================================================
static  GPIO_TypeDef * const GPIOInitTable[] = {
		GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOG, GPIOG,
		GPIOD, GPIOD, GPIOD, GPIOD, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE,
		GPIOD, GPIOD, GPIOD,
		GPIOB, GPIOB, GPIOC, GPIOE, GPIOE, GPIOF, GPIOG, GPIOG, GPIOG, GPIOG,
		0
};
static uint8_t const PINInitTable[] = {
		0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 0, 1,
		14, 15, 0, 1, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		8, 9, 10,
		5, 6, 0, 0, 1, 11, 4, 5, 8, 15,
		0
};
//=================================================================================================
// SDRAM_Init function
//=================================================================================================
void SDRAM_Init(void)
{

	volatile uint32_t i = 0;
	//set up GPIO lines

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;

	while(GPIOInitTable[i] != 0){
		gpio_conf(GPIOInitTable[i], PINInitTable[i],  MODE_AF, TYPE_PUSHPULL, SPEED_100MHz, PULLUP_NONE, 12);
		i++;
	}
	
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;   // enable FMC clock
	// Initialization step 1
	// We are actually initializing bank 6 
	// but some registers are for pairs of banks
	// this should turn write protection off
	FMC_Bank5_6->SDCR[0] = FMC_SDCR1_SDCLK_1  | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE_1; //set bursting and pipelining options - not sure how these are determined
	FMC_Bank5_6->SDCR[1] = FMC_SDCR1_NR_0	  | FMC_SDCR1_MWID_0 | FMC_SDCR1_NB | FMC_SDCR1_CAS;  // rows 12 bits 16bit wide 4 internal banks 3 cycle latency 8 column bits 
	// Initialization step 2
	// These terms do not match up exactly with what I found in the datasheet

	// http://www.issi.com/WW/pdf/42-45S16400J.pdf
	// but it appears that we can use slower timings and it should be OK
	// TRC =  row cycle 8 
	// TRP = row precharge 3 cycels
	// TMRD = load mode register to active 3 cycles (we only need 2)
	// TXSR = exit self  refresh 8 cycles 
	// TRAS = self refresh time 5 cycles
	// TWR = recovery delay 3 cycles
	// TRCD = row to column delay 3 cycles



	// I'm hoping this matches up with what I find in the datasheet
	FMC_Bank5_6->SDTR[0] = TRC(7)  | TRP(2);
	FMC_Bank5_6->SDTR[1] = TMRD(2) | TXSR(7) | TRAS(4) | TWR(2) | TRCD(2);
	// Initialization step 3
	// 2 auto refresh cycles - use bank2 (in this case 6) 
	// configure clock
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR 	 = 1 | FMC_SDCMR_CTB2 | (1 << 5);
	// Initialization step 4
	// some sort of delay - I may want to do a sleep here instead.
	// but it looks like the while loop may do this anyhow because 
	// i suspect this line is getting 'optimized' out.
	chThdSleepMilliseconds(10);
	//	for(i = 0; i  < 1000000; i++);
	// Initialization step 5
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	// all the above but . . . precharge all banks
	FMC_Bank5_6->SDCMR 	 = 2 | FMC_SDCMR_CTB2 | (1 << 5);
	// Initialization step 6
        // auto refresh and 5 refresh cycles
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR 	 = 3 | FMC_SDCMR_CTB2 | (8 << 5);	
	// Initialization step 7
        // loadmode register- bank2 (sequential with burst length of 2
	// cas latency of 3 
	// single location access
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	FMC_Bank5_6->SDCMR 	 = 4 | FMC_SDCMR_CTB2 | (1 << 5) | (0x231 << 9);
	// Initialization step 8
	// refresh count number of rows / refresh period
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	FMC_Bank5_6->SDRTR |= (683 << 1);
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
	// Clear SDRAM
	//for(ptr = SDRAM_BASE; ptr < (SDRAM_BASE + 0x100); ptr += 4)
	//    *ptr = 0xf000f000;
}
//=================================================================================================
// End of file
//=================================================================================================

