/*
 * delay.h
 *
 *  Created on: Sep 6, 2014
 *      Author: B51761
 */

#ifndef DELAY_H_
#define DELAY_H_
#include<stdint.h>
#include"MKL26Z4.h"

#define LOOP_CNT_1MS_24MHZ 2600
#define LOOP_CNT_1MS_48MHZ LOOP_CNT_1MS_24MHZ*2
#define LOOP_CNT_1MS_72MHZ LOOP_CNT_1MS_24MHZ*3
#define LOOP_CNT_1MS_96MHZ LOOP_CNT_1MS_24MHZ*4

void delay_busy(uint16_t msec);

void delay_low_power(uint16_t msec);

#endif /* DELAY_H_ */
