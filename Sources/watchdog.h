/*
 * watchdog.h
 *
 *  Created on: Sep 6, 2014
 *      Author: B51761
 */
#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include<MKL26Z4.h>
void watchdog_init();
void watchdog_reset();
void watchdog_service();


#endif /* DELAY_H_ */
