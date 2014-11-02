/*
 * watchdog.c
 *
 *  Created on: Sep 6, 2014
 *      Author: B51761
 */
#include"watchdog.h"

void watchdog_init()
{
	/*1024 LPO Clock*/
	SIM_COPC = SIM_COPC_COPT(0x03);
}
void watchdog_reset()
{
	/*reset counter of watchdog*/
	SIM_SRVCOP = SIM_SRVCOP_SRVCOP(0x55);
	SIM_SRVCOP = SIM_SRVCOP_SRVCOP(0xAA);
}
 
