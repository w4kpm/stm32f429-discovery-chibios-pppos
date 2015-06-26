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

mutex_t SD3mtx;

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
    chprintf((BaseSequentialStream*)&SD3,"pass %d\r\n",pass++);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOG, 13);     /* Green.  */
    chprintf((BaseSequentialStream*)&SD3,"pass %d\r\n",pass++);
    chThdSleepMilliseconds(500);
  }

	return MSG_OK;
}


/*
 * Shell related
 */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

static const ShellCommand shell_commands[] = {
	SYSINFO_SHELL_COMMANDS,
	{NULL, NULL}
};

static THD_WORKING_AREA(waEchoServerThread, 512 + 1024);
static THD_FUNCTION(EchoServerThread, arg) {
	(void) arg;
	int pass = 0;
  
	int sock;
	struct sockaddr_in sa;
	char buffer[1024];
	int recsize;
	socklen_t fromlen;

	chRegSetThreadName("EchoServerThread");

	sock = lwip_socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == -1) {
		return MSG_RESET;
	}

	memset(&sa, 0, sizeof sa);
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_port = htons(8000);
	fromlen = sizeof(sa);

	if (lwip_bind(sock, (struct sockaddr *) &sa, sizeof(sa)) == -1) {
		lwip_close(sock);
		return MSG_RESET;
	}

	while (!chThdShouldTerminateX()) {
		recsize = lwip_recvfrom(sock, buffer, sizeof(buffer), 0,
				(struct sockaddr *) &sa, &fromlen);
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

		lwip_sendto(sock, (void *) buffer, recsize, 0, (struct sockaddr *) &sa,
				fromlen);
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

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));

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

		// Make sure connection is stable
		while (connected < 5) {
			chThdSleep(MS2ST(100));
			if (connected == 0) { // reset by pppThread while waiting for stable connection
			    chprintf((BaseSequentialStream*)&SD3,"Close Connection - not stable\r\n");
			    chThdSleepMilliseconds(100);

				pppClose(pd);
				break;
			}
			connected++;
		}

		// Run server threads
		echoServerThread = chThdCreateStatic(waEchoServerThread,
				sizeof(waEchoServerThread), NORMALPRIO + 1, EchoServerThread, NULL);
		shellServerThread = chThdCreateStatic(waShellServerThread,
				sizeof(waShellServerThread), NORMALPRIO + 1, ShellServerThread,
				NULL);

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
