/*
 * sysinfo.h
 *
 *  Created on: Sep 17, 2014
 *      Author: Douglas
 */

#ifndef SYSINFO_H_
#define SYSINFO_H_

#ifdef __cplusplus
extern "C" {
#endif

void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#define SYSINFO_SHELL_COMMANDS \
	  {"mem", cmd_mem}, \
	  {"threads", cmd_threads}

#endif /* SYSINFO_H_ */
