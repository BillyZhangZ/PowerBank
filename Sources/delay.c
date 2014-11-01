/*
 * delay.c
 *
 *  Created on: Sep 6, 2014
 *      Author: B51761
 */
#include"delay.h"
/*
 * assume execute in core, 0.84 dmips/mhz
 */
void delay_busy(uint16_t msec)
{
	int volatile loop_cnt = 0;
	switch((MCG_C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT)
	{
		/*24MHZ core*/
		case 0x00:
			loop_cnt = LOOP_CNT_1MS_24MHZ*msec;
			while(--loop_cnt);
			break;
		/*48MHZ core*/			
		case 0x01:
			loop_cnt = LOOP_CNT_1MS_48MHZ*msec;
			while(--loop_cnt);
			break;
		/*72MHZ core*/
		case 0x02:
			loop_cnt = LOOP_CNT_1MS_72MHZ*msec;
			while(--loop_cnt);
			break;
		/*96MHZ core*/
		case 0x03:
			loop_cnt = LOOP_CNT_1MS_96MHZ*msec;
			while(--loop_cnt);
			break;
		default:
			break;
	}
}

void delay_low_power(uint16_t msec)
{
	
}
