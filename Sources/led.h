/*
 * led.h
 *
 *  Created on: Sep 4, 2014
 *      Author: B51761
 */

#ifndef LED_H_
#define LED_H_
#include"MKL26Z4.h"

void led_init(void);
void led_deinit(void);
void led_on(uint8_t);
void led_off(uint8_t);
void led_toggle(uint8_t num);
void led_ctrl(int led1, int led2, int led3, int led4);
#endif /* LED_H_ */
