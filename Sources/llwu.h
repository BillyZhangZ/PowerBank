/*
 * llwu.h
 *
 *  Created on: Sep 5, 2014
 *      Author: B51761
 */

#ifndef LLWU_H_
#define LLWU_H_
#include<stdint.h>
#include"MKL26Z4.h"

#if 0
#define USB_HOST_PLUGIN 0x01
#define SWITCH_PRESSED 0x02
#define INTERNAL_TIMER 0x04
#define USB_DEVICE_PLUGIN 0x08
#endif
void enter_lls(void);
void LLW_IRQHandler();
void llwu_init(void);
void mask_switch(void);
void unmask_switch(void);
#endif /* LLWU_H_ */
