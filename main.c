/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "lwip/sockets.h"
#include "lwip/tcpip.h"

#include "socketstreams.h"

#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "chprintf.h"
#include "shell.h"
#include "sysinfo.h"
#include "lwip/sockets.h"
#include <string.h>
#include "ppp/ppp.h"
#include "stm32f4xx.h"

//#include "lwipthread.h"
static uint8_t txbuf[2];
static uint8_t rxbuf[2];
static char text[255];

static const SPIConfig std_spicfg1 = {
  NULL,
  GPIOC,                                                        /*port of CS  */
  2,                                                /*pin of CS   */
  SPI_CR1_CPOL|SPI_CR1_CPHA| \
  SPI_CR1_SPE|SPI_CR1_MSTR|SPI_CR1_BIDIMODE ,
  NULL
};

/*
The interface to the LCD controller appears to be bidirectional - and you have to do some stuff to set it up before it will take parallel info from the lcd controller from STM32 - it may be best to work on connecting to VNC server and speed issues before messing with the screwy SPI stuff with this display.
*/


void init_spi()
{
//  palSetPadMode(GPIOA, CS, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOA, CS2, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOB, CK, PAL_MODE_OUTPUT_PUSHPULL);
    // palSetPadMode(GPIOB, MISO, PAL_MODE_INPUT_PULLDOWN);
    //palSetPadMode(GPIOB, MOSI, PAL_MODE_OUTPUT_PUSHPULL);
    //palSetPad(GPIOA,CS);
    //palSetPad(GPIOA,CS2);
    // palClearPad(GPIOB,CK);

}
uint8_t spi_read(device,location)
{
  spiStart(&SPID5,device);
  spiSelect(&SPID5);
  txbuf[0] = 0x80 | location;
  spiSend(&SPID5,1,&txbuf);
  spiReceive(&SPID5,1,&rxbuf);
  spiUnselect(&SPID5);
  spiStop(&SPID5);
  return(rxbuf[0]);
}
void spi_write(device,location,data)
{
  spiStart(&SPID5,device);
  spiSelect(&SPID5);
  txbuf[0] = 0x00 | location;
  txbuf[1] = data;
  spiSend(&SPID5,2,&txbuf);
  spiUnselect(&SPID5);
  spiStop(&SPID5);
}





mutex_t SD3mtx;
void TFTLCD_Init(void);
/* stolen from a version of sys_arch I found on wizhippo's github 

https://github.com/wizhippo/stm32f4-chibios-lwip-pppos.git
*/
u32_t sys_jiffies(void) {
	static u32_t jiffies = 0;
#if OSAL_ST_FREQUENCY == 1000
	jiffies += (u32_t)osalOsGetSystemTimeX();
#elif (OSAL_ST_FREQUENCY / 1000) >= 1 && (OSAL_ST_FREQUENCY % 1000) == 0
	jiffies += ((u32_t)osalOsGetSystemTimeX() - 1) / (OSAL_ST_FREQUENCY / 1000) + 1;
#elif (1000 / OSAL_ST_FREQUENCY) >= 1 && (1000 % OSAL_ST_FREQUENCY) == 0
	jiffies += ((u32_t)osalOsGetSystemTimeX() - 1) * (1000 / OSAL_ST_FREQUENCY) + 1;
#else
	jiffies += (u32_t)(((u64_t)(osalOsGetSystemTimeX() - 1) * 1000) / OSAL_ST_FREQUENCY) + 1;
#endif

	return jiffies;
}


#define LCD_WIDTH	 320
#define LCD_HEIGHT	240

#define HFP   16
#define HSYNC 96
#define HBP   48

#define VFP   10
#define VSYNC 2
#define VBP   33

#define ACTIVE_W (HSYNC + LCD_WIDTH + HBP - 1)
#define ACTIVE_H (VSYNC + LCD_HEIGHT + VBP - 1)

#define TOTAL_WIDTH  (HSYNC + HBP + LCD_WIDTH + HFP - 1)
#define TOTAL_HEIGHT (VSYNC + VBP + LCD_HEIGHT + VFP - 1)
#define PIXELWIDHT 2
static SerialConfig uartCfg =
{
    460800,// bit rate
    0,
    0,
    0,
};
/*
 * Application entry point.
 */


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  int pass = 0;
  chRegSetThreadName("Thread1");

  while (TRUE) {
    palSetPad(GPIOG, 13);       /* Green.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOG, 13);     /* Green.  */
    chThdSleepMilliseconds(500);
  }

	return MSG_OK;
}


#ifndef SOCK_TARGET_HOST
#define SOCK_TARGET_HOST  "172.30.1.98"
#endif

#ifndef SOCK_TARGET_PORT
#define SOCK_TARGET_PORT  5901
#endif



/*
 * Shell related
 */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

static const ShellCommand shell_commands[] = {
	SYSINFO_SHELL_COMMANDS,
	{NULL, NULL}
};

char vncbuffer[1024];
int vncsocket;

void log_data(char* text)
{
    chprintf((BaseSequentialStream*)&SD3,text);
}

void write_byte(uint8_t x){
    int numbytes;
    numbytes=lwip_send(vncsocket,&x,1,0);
    if (numbytes != 1)
	log_data("writeByte Incorrect length\r\n");
}

void write_string(char *str)
{
    int numbytes;
    numbytes=lwip_send(vncsocket,str,strlen(str),0);
    if (numbytes != strlen(str))
	log_data("writeString Incorrect length\r\n");

}

void write_16(uint16_t data)
{
    int numbytes;
    numbytes=lwip_send(vncsocket,&data,2,0);
    if (numbytes != 2)
	log_data("write16 Incorrect length\r\n");

}


void write_32(uint32_t data)
{
    int numbytes;
    numbytes=lwip_send(vncsocket,&data,4,0);
    if (numbytes != 4)
	log_data("write32 Incorrect length\r\n");

}



void read_all_data(void)
{
    int x;
    int numbytes;
    numbytes=lwip_recv(vncsocket,&vncbuffer,1024,0);
    vncbuffer[numbytes] = 0;
    for (x=0;x<numbytes;x++)
	chprintf((BaseSequentialStream*)&SD3,"0x%X ",vncbuffer[x]);
    chprintf((BaseSequentialStream*)&SD3,"\r\n");


}



void framebuffer_request(uint16_t x, 
			 uint16_t y, 
			 uint16_t width, 
			 uint16_t height, 
			 uint8_t incremental)
{
    read_all_data();
    vncbuffer[0] = 3;
    vncbuffer[1] = incremental;
    midptr = vncbuffer+2;
    *midptr = x;
    midptr = vncbuffer+4;
    *midptr = y;
    midptr = vncbuffer+6;
    *midptr = width;
    midptr = vncbuffer+8;
    *midptr = height;
    x = lwip_send(vncsocket,&vncbuffer,10,0);
    if (x != 10)
	log_data("set Encodings Incorrect length\r\n");
    process_frame_response();
}



void set_encodings()
{
    int x;
    uint16_t *midptr;
    uint32_t *longptr;
    
    vncbuffer[0] = 2;
    vncbuffer[1] = 0;
    midptr = vncbuffer+2;
    *midptr = 3;
    longptr = vncbuffer+4;
    *longptr = 5;
    longptr = vncbuffer+8;
    *longptr = 1;
    longptr = vncbuffer+12;
    *longptr = 0;

    for (x=0;x<16;x++)
	chprintf((BaseSequentialStream*)&SD3,"0x%X ",vncbuffer[x]);
    chprintf((BaseSequentialStream*)&SD3,"\r\n");
    x = lwip_send(vncsocket,&vncbuffer,16,0);
    if (x != 16)
	log_data("set Encodings Incorrect length\r\n");
	
    
}

int connect_vnc(void){
    write_string("RFB 003.003\n");
    write_byte(1);
    set_encodings();

}





static THD_WORKING_AREA(waVncThread, 2048 );
static THD_FUNCTION(VncThread, arg) {

	int ret;
	int x;
	struct sockaddr_in sa;

	int recsize;
	socklen_t cli_addr_len;
	struct sockaddr_in cli_addr;

	socklen_t fromlen;

	chRegSetThreadName("EchoServerThread");
	chprintf((BaseSequentialStream*)&SD3,"Shell Server Starting \r\n");

	vncsocket = lwip_socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);
	if (vncsocket == -1) {
	    chprintf((BaseSequentialStream*)&SD3,"Closing A - no socket \r\n");

		return MSG_RESET;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = PP_HTONS(SOCK_TARGET_PORT);
	sa.sin_addr.s_addr = inet_addr(SOCK_TARGET_HOST);
	fromlen = sizeof(sa);
	ret = lwip_connect(vncsocket, (struct sockaddr*)&sa, sizeof(sa));
	chprintf((BaseSequentialStream*)&SD3,"Returned '%d' \r\n",ret);
	connect_vnc();
}


static THD_WORKING_AREA(waEchoServerThread, 2048 );
static THD_FUNCTION(EchoServerThread, arg) {
	(void) arg;
	int pass = 0;
  
	int sock,newsockfd;
	struct sockaddr_in sa;
	char buffer[1024];
	int recsize;
	socklen_t cli_addr_len;
	struct sockaddr_in cli_addr;

	socklen_t fromlen;

	chRegSetThreadName("EchoServerThread");
	chprintf((BaseSequentialStream*)&SD3,"Shell Server Starting \r\n");

	sock = lwip_socket(AF_INET,  SOCK_STREAM, IPPROTO_TCP);
	if (sock == -1) {
	    chprintf((BaseSequentialStream*)&SD3,"Closing A - no socket \r\n");

		return MSG_RESET;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_port = PP_HTONS(26);
	fromlen = sizeof(sa);

	if (lwip_bind(sock, (struct sockaddr *) &sa, sizeof(sa)) == -1) {
	    chprintf((BaseSequentialStream*)&SD3,"Closing B - no BIND \r\n");
		lwip_close(sock);
		return MSG_RESET;
	}
	if (lwip_listen(sock, 1) < 0) {
		lwip_close(sock);
		return MSG_RESET;
	}

	while (!chThdShouldTerminateX()) {
	    chprintf((BaseSequentialStream*)&SD3,"Attempting to rx\r\n");
	    newsockfd = lwip_accept(sock, (struct sockaddr *) &cli_addr,
				    &cli_addr_len);
	    if (newsockfd < 0) {
		break;
	    }
	    chprintf((BaseSequentialStream*)&SD3,"Accepted\r\n");
	    chThdSleepMilliseconds(100);
	    while (!chThdShouldTerminateX()) {

		recsize = lwip_recvfrom(newsockfd, buffer, sizeof(buffer), 0,
				(struct sockaddr *) &sa, &fromlen);
		chprintf((BaseSequentialStream*)&SD3,"recsize %d - fromlen %d\r\n",recsize,fromlen);
		if (recsize < 0) {
			break;
		}
		pass ++;
		if (pass%2 ==0)
		    {
			palClearPad(GPIOG, 14);
		    }
		else
		    {
			palSetPad(GPIOG, 14);
		    }

		lwip_sendto(newsockfd, (void *) buffer, recsize, 0, (struct sockaddr *) &sa,
				fromlen);
	    }
	    lwip_close(sock);
	}

	lwip_close(sock);

	return MSG_OK;
}


/*
 * TCP Shell server thread
 */

static THD_WORKING_AREA(waShellServerThread, 512);
static THD_FUNCTION(ShellServerThread, arg) {
	(void) arg;

	int sockfd, newsockfd;
	socklen_t cli_addr_len;
	struct sockaddr_in serv_addr, cli_addr;

	SocketStream sbp;
	ShellConfig shell_cfg;
	thread_t *shelltp;

	chRegSetThreadName("ShellServerThread");
	chprintf((BaseSequentialStream*)&SD3,"Shell Server Starting \r\n");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = PP_HTONS(25);

	sockfd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sockfd < 0)
		return MSG_RESET;

	if (lwip_bind(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		lwip_close(sockfd);
		return MSG_RESET;
	}

	if (lwip_listen(sockfd, 1) < 0) {
		lwip_close(sockfd);
		return MSG_RESET;
	}

	cli_addr_len = sizeof(cli_addr);

	while (!chThdShouldTerminateX()) {
		newsockfd = lwip_accept(sockfd, (struct sockaddr *) &cli_addr,
				&cli_addr_len);
		if (newsockfd < 0) {
			break;
		}

		ssObjectInit(&sbp, newsockfd);
		shell_cfg.sc_channel = (BaseSequentialStream*) &sbp;
		shell_cfg.sc_commands = shell_commands;

		shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO+2);
		chThdWait(shelltp);

		lwip_close(newsockfd);
	}

	if (lwip_shutdown(sockfd, SHUT_RDWR) < 0) {
		// oops
	}
	lwip_close(sockfd);

	return MSG_OK;
}


static THD_WORKING_AREA(waShellServerThread2, 512);
static THD_FUNCTION(ShellServerThread2, arg) {
	(void) arg;


	ShellConfig shell_cfg;
	thread_t *shelltp;

	chRegSetThreadName("ShellServerThread2");

	shell_cfg.sc_channel = (BaseSequentialStream*) &SD3;
	shell_cfg.sc_commands = shell_commands;

	shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO+2);
		chThdWait(shelltp);


	return MSG_OK;
}

/*
 * Handle link status events
 */
static void ppp_linkstatus_callback(void *ctx, int errCode, void *arg) {
	(void) arg;

	volatile int *connected = (int*)ctx;
	chprintf((BaseSequentialStream*)&SD3,"callback %x\r\n",errCode);
	chThdSleepMilliseconds(100);

	if (errCode == PPPERR_NONE) {
		*connected = 1;
	} else {
		*connected = 0;
	}
}


int main(void) {
  unsigned i;
  thread_t *echoServerThread = 0;
  thread_t *shellServerThread = 0;
  thread_t *shellServerThread2 = 0;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  TFTLCD_Init();
  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
      RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
      RCC_AHB1ENR_GPIOGEN;

  chMtxObjectInit(&SD3mtx);
  sdStart(&SD1, &uartCfg);
  sdStart(&SD3, &uartCfg);

  palSetPadMode(GPIOG, 13, PAL_MODE_OUTPUT_PUSHPULL); 
  palSetPadMode(GPIOG, 14, PAL_MODE_OUTPUT_PUSHPULL );
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);  
  printf("test from printf \r\n");
  chprintf((BaseSequentialStream*)&SD3,"After start thread\r\n");
  chThdSleepMilliseconds(100);


  palSetPad(GPIOG, 14);

  chprintf((BaseSequentialStream*)&SD3,"After set pad Init\r\n");
  chThdSleepMilliseconds(100);


  tcpip_init(NULL, NULL);
  chprintf((BaseSequentialStream*)&SD3,"After tcp Init\r\n");
  chThdSleepMilliseconds(100);



  pppInit();
  chprintf((BaseSequentialStream*)&SD3,"After PPP Init\r\n");
  chThdSleepMilliseconds(100);




  shellInit();
  chprintf((BaseSequentialStream*)&SD3,"After Shell Init\r\n");
  chThdSleepMilliseconds(100);
  chThdSetPriority(PPP_THREAD_PRIO + 1);

  chprintf((BaseSequentialStream*)&SD3,"After set Priority\r\n");
  chThdSleepMilliseconds(100);

	while (TRUE) {
		volatile int connected = 0;

		int pd = pppOverSerialOpen(&SD1, ppp_linkstatus_callback,
				(int*) &connected);

		if (pd < 0) {
			chThdSleep(MS2ST(100));
			continue;
		}
		chprintf((BaseSequentialStream*)&SD3,"After ppp open\r\n");
		chThdSleepMilliseconds(100);


		// Wait for initial connection
		int timeout = 0;

		while (connected < 1) {
			chThdSleep(MS2ST(500));
			if(timeout++ > 10) {  // If we waited too long restart connection
			    chprintf((BaseSequentialStream*)&SD3,"Close Connection - too long\r\n");
			    chThdSleepMilliseconds(100);

				pppClose(pd);
				break;
			}
			
		}
		chprintf((BaseSequentialStream*)&SD3,"entering connection \r\n");

		// Make sure connection is stable
		while (connected < 5) {
			chThdSleep(MS2ST(100));
			printf("Connected\r\n");
			if (connected == 0) { // reset by pppThread while waiting for stable connection
			    chprintf((BaseSequentialStream*)&SD3,"Close Connection - not stable\r\n");
			    chThdSleepMilliseconds(100);

				pppClose(pd);
				break;
			}
			connected++;
		}

		// Run server threads

		echoServerThread = chThdCreateStatic(waVncThread,
						     sizeof(waVncThread), NORMALPRIO + 1, VncThread, NULL);

		//		echoServerThread = chThdCreateStatic(waEchoServerThread,
		//						     sizeof(waEchoServerThread), NORMALPRIO + 1, EchoServerThread, NULL);
		//		shellServerThread = chThdCreateStatic(waShellServerThread,
		//						      sizeof(waShellServerThread), NORMALPRIO + 1, ShellServerThread,
		//						NULL);

		/*		shellServerThread2 = chThdCreateStatic(waShellServerThread2,
				sizeof(waShellServerThread2), NORMALPRIO + 1, ShellServerThread2,
				NULL);*/

		while (connected > 0) {
			chThdSleep(MS2ST(200));
		}

		// Tear down threads
		chThdTerminate(echoServerThread);
		chThdWait(echoServerThread);
		chThdTerminate(shellServerThread);
		chThdWait(shellServerThread);
	}

  return 0;
}
